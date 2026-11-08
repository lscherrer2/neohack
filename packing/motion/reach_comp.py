"""Reach compensation: aim higher the further out the target is (optional, Calibration tab toggle).

The user saw the arm land low far out even after the desk-plane and joint-offset fits (2026-11-07) and asked
for an empirical "extra sag correction": raise every aim point by

    lift(r) = lift_mm · (r / (at_reach · max_reach)) ** power

r = the tool point's horizontal distance from the swivel, max_reach = the farthest the tool point gets out
(`max_reach_m`). Defaults: 6 mm at 70 % of max reach, power 2 — sag grows with the lever arm, so
near the base it's ~0 (where touches already read right) and it rises steeply further out.

Applied at RUN time only (`raise_steps`, called by `shop/goto_runner.py` and `boxes/packer.py` right before
`program.run`): plans, their checks and the dry-run path stay in true positions; only the commanded joint poses
are re-solved with the target z raised (same tool orientation). Hand-guided calibrations (tray, boxes, desk —
motors off, no sag) are untouched. Settings: `cell.yaml` `reach_comp:` defaults + the toggle / height saved in
`data/reach_comp.json` (sim: `sim_reach_comp.json`).
"""

from __future__ import annotations

import dataclasses
import json
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from ..joints import BODY

MAX_IK_ERR_M = 0.002  # a raised pose must be reached this closely, else the run is refused before moving


@dataclass
class ReachComp:
    enabled: bool = False
    lift_mm: float = 6.0  # how much higher at at_reach of max reach (user's pick after the averaged tray fit, 2026-11-07)
    at_reach: float = 0.7  # fraction of max reach where lift_mm applies
    power: float = 2.0  # 1 = straight line from the swivel, 2 = grows with the lever arm

    def to_dict(self) -> dict:
        return dataclasses.asdict(self)


def load(path: str | Path | None, defaults: ReachComp) -> ReachComp:
    """Defaults from cell.yaml, overridden by the saved toggle / values."""
    if not path or not Path(path).exists():
        return dataclasses.replace(defaults)
    raw = json.loads(Path(path).read_text())
    known = {f.name for f in dataclasses.fields(ReachComp)}
    return dataclasses.replace(defaults, **{k: v for k, v in raw.items() if k in known})


def save(path: str | Path, rc: ReachComp) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(rc.to_dict(), indent=2))


def max_reach_m(model) -> float:
    """Farthest horizontal distance of the tool point from the swivel over the joint ranges (cached per model)."""
    cached = getattr(model, "_max_reach_m", None)
    if cached is not None:
        return cached
    best = 0.0
    grid = np.linspace(-100, 100, 41)
    for lift in grid:
        for elbow in grid:
            for wrist in grid[::4]:
                q = {"shoulder_pan": 0.0, "shoulder_lift": lift, "elbow_flex": elbow, "wrist_flex": wrist,
                     "wrist_roll": 0.0}
                best = max(best, radius(model, model.tip(q)))
    model._max_reach_m = best
    return best


def radius(model, p) -> float:
    return float(np.linalg.norm(np.asarray(p)[:2] - model.pan_xy))


def lift_m(model, rc: ReachComp, r: float) -> float:
    """How much higher (m) to aim at horizontal distance r (0 when disabled)."""
    if not rc.enabled or rc.lift_mm == 0:
        return 0.0
    ref = rc.at_reach * max_reach_m(model)
    return rc.lift_mm / 1000 * (max(r, 0.0) / ref) ** rc.power


PLANAR = ("shoulder_lift", "elbow_flex", "wrist_flex")  # pan and roll stay: a pure lift keeps the arm's plane


MAX_TILT_CHANGE_DEG = 5.0  # near a singularity the tool may tilt a little to reach the raised point; more = refused
TILT_WEIGHTS = (0.05, 0.005)  # m per unit of rotation-matrix difference: tilt held firmly first; if that can't
# reach the point (near a singularity), position first and tilt second (~0.1 mm per degree)
TILT_WEIGHT = TILT_WEIGHTS[-1]


def _tilt_change_deg(R0: np.ndarray, R1: np.ndarray) -> float:
    return float(np.degrees(np.arccos(np.clip((np.trace(R0.T @ R1) - 1) / 2, -1.0, 1.0))))


def _raise_planar(model, body: dict, T: np.ndarray, target: np.ndarray, iters: int = 60,
                  tilt_weight: float = TILT_WEIGHT) -> tuple[dict, float, float]:
    """Raise the tool point with lift/elbow/wrist only (pan and roll unchanged): damped least squares on FK.
    Position must be hit; the tool tilt is kept as well as possible (soft). Away from singularities both are met
    exactly. Near full stretch all three joints tilt the tool alike, so keeping the tilt EXACTLY makes a few mm
    straight up impossible — the old orientation-weighted placo IK stalled 2-10 mm short there and the run was
    refused as "out of reach". Returns (pose, tip error m, tilt change deg)."""
    R0 = T[:3, :3]
    q = dict(body)

    def residual(p):
        F = model.fk(p)
        return np.concatenate([F[:3, 3] - target, tilt_weight * (F[:3, :3] - R0).ravel()])

    h, lam = 0.02, 1e-6  # finite-difference step (units); LM damping
    r = residual(q)
    cost = float(r @ r)
    for _ in range(iters):
        if cost < 1e-10:
            break
        J = np.empty((r.size, len(PLANAR)))
        for i, j in enumerate(PLANAR):
            J[:, i] = (residual({**q, j: q[j] + h}) - r) / h
        A, g = J.T @ J, J.T @ r
        step = -np.linalg.solve(A + lam * np.eye(len(PLANAR)), g)
        step = np.clip(step, -5.0, 5.0)  # units per iteration: no wild jumps
        if np.abs(step).max() < 1e-5:
            break
        # stay inside the joint limits: a joint at its limit stops there and the other two make up the rest
        trial = {**q, **{j: float(np.clip(q[j] + s, -100.0, 100.0)) for j, s in zip(PLANAR, step)}}
        rt = residual(trial)
        if float(rt @ rt) < cost:
            q, r, cost, lam = trial, rt, float(rt @ rt), max(lam / 3, 1e-9)
        else:
            lam *= 10
            if lam > 1e3:
                break
    F = model.fk(q)
    return q, float(np.linalg.norm(F[:3, 3] - target)), _tilt_change_deg(R0, F[:3, :3])


def raise_pose(model, rc: ReachComp, q: dict) -> dict:
    """Joint pose whose tool point is lift(r) higher than q's (same orientation). ValueError if unreachable."""
    body = {j: float(q[j]) for j in BODY}
    T = model.fk(body)
    dz = lift_m(model, rc, radius(model, T[:3, 3]))
    if dz < 1e-4:
        return dict(q)
    target = T.copy()
    target[2, 3] += dz
    for w in TILT_WEIGHTS:
        raised, err, tilt = _raise_planar(model, body, T, target[:3, 3], tilt_weight=w)
        if err <= MAX_IK_ERR_M / 4:
            break
    over = max(abs(v) for v in raised.values()) > 100
    if err > MAX_IK_ERR_M or over or tilt > MAX_TILT_CHANGE_DEG:
        why = ("a joint would pass its limit" if over else
               f"the tool would have to tilt {tilt:.0f}°" if err <= MAX_IK_ERR_M else f"IK {err * 1000:.1f} mm off")
        raise ValueError(f"reach compensation: can't aim {dz * 1000:.0f} mm higher at "
                         f"{np.round(T[:3, 3] * 1000).tolist()} mm ({why}) — turn it off or lower it")
    return {**q, **raised}


def raise_steps(model, steps: list, rc: ReachComp) -> list:
    """The same program with every commanded pose raised by lift(r). Unchanged (same list) when disabled."""
    from ..motion.program import Grip, JointMove, LineMove

    if not rc.enabled or rc.lift_mm == 0:
        return steps
    out = []
    for st in steps:
        if isinstance(st, JointMove):
            out.append(dataclasses.replace(st, pose=raise_pose(model, rc, st.pose)))
        elif isinstance(st, LineMove):
            out.append(dataclasses.replace(st, path=[raise_pose(model, rc, q) for q in st.path]))
        elif isinstance(st, Grip) and st.on_fail:
            out.append(dataclasses.replace(st, on_fail=raise_steps(model, st.on_fail, rc)))
        else:
            out.append(st)
    return out


def table(model, rc: ReachComp) -> list[dict]:
    """Lift at a few reach fractions, for the UI."""
    m = max_reach_m(model)
    on = dataclasses.replace(rc, enabled=True)
    return [{"reach": f, "r_mm": round(f * m * 1000), "lift_mm": round(lift_m(model, on, f * m) * 1000, 1)}
            for f in (0.3, 0.5, 0.7, 0.9)]
