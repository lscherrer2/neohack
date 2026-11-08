"""Reach compensation: aim higher the further out the target is (optional, Calibration tab toggle).

The user saw the arm land low far out even after the desk-plane and joint-offset fits (2026-11-07) and asked
for an empirical "extra sag correction": raise every aim point by

    lift(r) = lift_mm · (r / (at_reach · max_reach)) ** power

r = the tool point's horizontal distance from the swivel, max_reach = the farthest the tool point gets out
(`max_reach_m`). Defaults: ½ in (12.7 mm) at 70 % of max reach, power 2 — sag grows with the lever arm, so
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
ORIENTATION_WEIGHT = 0.5  # keep the tool orientation (a level box grip stays level) while raising


@dataclass
class ReachComp:
    enabled: bool = False
    lift_mm: float = 12.7  # how much higher at at_reach of max reach
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


def raise_pose(model, rc: ReachComp, q: dict) -> dict:
    """Joint pose whose tool point is lift(r) higher than q's (same orientation). ValueError if unreachable."""
    body = {j: float(q[j]) for j in BODY}
    T = model.fk(body)
    dz = lift_m(model, rc, radius(model, T[:3, 3]))
    if dz < 1e-4:
        return dict(q)
    target = T.copy()
    target[2, 3] += dz
    raised, err = model.ik(body, target, orientation_weight=ORIENTATION_WEIGHT)
    if err > MAX_IK_ERR_M or any(abs(v) > 100 for v in raised.values()):
        raise ValueError(f"reach compensation: can't aim {dz * 1000:.0f} mm higher at "
                         f"{np.round(T[:3, 3] * 1000).tolist()} mm (out of reach) — turn it off or lower it")
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
