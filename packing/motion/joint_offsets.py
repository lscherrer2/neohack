"""Kinematic zero offsets fitted from desk touches (plan `.claude/plans/joint_offset_fit.md`).

The kinematics zero is the calibration pose, set by eye. A small zero error on `shoulder_lift` / `elbow_flex`
makes the computed tool height wrong by an amount that grows with reach — the same left and right of the arm
(user, 2026-11-07, 7 desk touches: right on the desk near the base, higher and higher further out). A plane can't
absorb that, but the touches can solve for it: they all lie on one flat desk, so find the joint angle corrections
(+ the desk plane) that make the jaw-tip points flattest.

    residual_i = z_i(Δ) − (a·x_i(Δ) + b·y_i(Δ) + c)       (jaw tip from FK with the offsets, mm)

Weak priors (offsets σ PRIOR_DEG, desk tilt σ TILT_PRIOR_DEG) keep it stable. The wrist isn't fitted: with the
gripper pointing down a wrist offset moves the tip sideways, hardly up or down.

Offsets are degrees added to each joint's URDF angle (`ArmModel(offsets_deg=...)`). They're saved per arm in
`calibration/<robot_id>_joint_offsets.json` (`cell.yaml` `kinematics.offsets_file`).
"""

from __future__ import annotations

import json
import math
import time
from pathlib import Path

import numpy as np

from ..joints import BODY

FIT_JOINTS = ("shoulder_lift", "elbow_flex")
PRIOR_DEG = 3.0  # offsets this big count as much as 1 mm of misfit
TILT_PRIOR_DEG = 1.0  # same for the desk tilt (the base stands on the desk: expect it near level)
NOISE_MM = 1.0  # touch noise the residuals are scaled by
MIN_POINTS = 6
MIN_RADIUS_SPAN_M = 0.08  # touches must span this much distance from the swivel (near AND far)...
MIN_AZIMUTH_SPAN_DEG = 40.0  # ...and this much around it (else a desk tilt looks like a reach error)
MAX_STD_DEG = 0.5  # an offset less certain than this isn't applied
MAX_OFFSET_DEG = 5.0  # bigger corrections mean something else is wrong (bad touch, wrong calibration)


class OffsetFitError(ValueError):
    """Not enough / badly spread touches (message shown to the user)."""


def load_offsets(path: str | Path | None) -> dict[str, float]:
    """Saved joint offsets (deg), {} if none."""
    if not path or not Path(path).exists():
        return {}
    raw = json.loads(Path(path).read_text())
    return {j: float(v) for j, v in (raw.get("offsets_deg") or {}).items() if j in BODY}


def save_offsets(path: str | Path, offsets: dict[str, float], fit: dict | None = None) -> Path:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps({"offsets_deg": {j: round(float(v), 4) for j, v in offsets.items()},
                                "created": time.strftime("%Y-%m-%dT%H:%M:%S"),
                                "source": "desk-touch fit (packing/motion/joint_offsets.py)",
                                "fit": fit or {}}, indent=2))
    return path


def jaw_tips(model, poses: list[dict], delta: dict[str, float] | None = None) -> np.ndarray:
    """Jaw-tip points (m) of joint poses, with `delta` (deg) added on top of the model's own offsets."""
    from ..boxes.desk import touch_point

    d = np.array([(delta or {}).get(j, 0.0) for j in BODY])
    return np.array([touch_point(np.asarray(model.kin.forward_kinematics(model.to_deg(q) + d))) for q in poses])


def _plane(P: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Least-squares plane (a, b, c) through points and the residuals (m)."""
    A = np.column_stack([P[:, 0], P[:, 1], np.ones(len(P))])
    abc, *_ = np.linalg.lstsq(A, P[:, 2], rcond=None)
    return abc, P[:, 2] - A @ abc


def _tilt_deg(a: float, b: float) -> float:
    return math.degrees(math.atan(math.hypot(a, b)))


def fit_offsets(model, poses: list[dict], joints: tuple[str, ...] = FIT_JOINTS) -> dict:
    """Fit joint offset corrections (deg, on top of the model's current offsets) from desk-touch joint poses.
    Returns a report: delta_deg, total_deg, std_deg, rms_before_mm / rms_after_mm, tilt before/after, per-point
    residuals vs distance from the swivel, ok + problems. Raises OffsetFitError if the touches can't determine it.

    Seen only through desk heights, lift and elbow both mostly tilt the reach (correlated ~0.8): if the pair isn't
    well determined, `shoulder_lift` alone is fitted instead (it still flattens the height-vs-reach pattern)."""
    rep = _fit(model, poses, joints)
    if not rep["ok"] and len(joints) > 1 and joints[0] == "shoulder_lift" and \
            any("isn't well determined" in p for p in rep["problems"]):
        alone = _fit(model, poses, ("shoulder_lift",))
        if alone["ok"]:
            alone["note"] = (f"{', '.join(joints[1:])} can't be told apart from shoulder_lift with these touches "
                             f"(both: {rep['problems'][0]}) — fitted shoulder_lift alone")
            return alone
    return rep


def _fit(model, poses: list[dict], joints: tuple[str, ...]) -> dict:
    from scipy.optimize import least_squares

    poses = [{j: float(q[j]) for j in BODY} for q in poses]
    if len(poses) < MIN_POINTS:
        raise OffsetFitError(f"touch at least {MIN_POINTS} spots for the joint fit ({len(poses)} so far) — "
                             f"near the arm and far out, left, middle and right")
    P0 = jaw_tips(model, poses)
    d = P0[:, :2] - model.pan_xy
    r = np.hypot(d[:, 0], d[:, 1])
    az = np.degrees(np.arctan2(d[:, 1], d[:, 0]))
    if r.max() - r.min() < MIN_RADIUS_SPAN_M:
        raise OffsetFitError(f"the touches are all about as far from the arm ({r.min() * 100:.0f}–{r.max() * 100:.0f} "
                             f"cm) — add some close in and some far out (≥ {MIN_RADIUS_SPAN_M * 100:.0f} cm apart)")
    if az.max() - az.min() < MIN_AZIMUTH_SPAN_DEG:
        raise OffsetFitError(f"the touches only cover {az.max() - az.min():.0f}° around the arm — spread them left, "
                             f"middle and right (≥ {MIN_AZIMUTH_SPAN_DEG:.0f}°)")

    abc0, res0 = _plane(P0)
    n = len(joints)
    tilt_sigma = math.tan(math.radians(TILT_PRIOR_DEG))

    def residuals(x):
        P = jaw_tips(model, poses, dict(zip(joints, x[:n])))
        a, b, c = x[n:]
        meas = (P[:, 2] - (a * P[:, 0] + b * P[:, 1] + c)) * 1000 / NOISE_MM
        return np.concatenate([meas, x[:n] / PRIOR_DEG, [a / tilt_sigma, b / tilt_sigma]])

    sol = least_squares(residuals, np.concatenate([np.zeros(n), abc0]), x_scale=[1.0] * n + [0.01, 0.01, 0.01])
    delta = dict(zip(joints, (float(v) for v in sol.x[:n])))
    P1 = jaw_tips(model, poses, delta)
    abc1 = sol.x[n:]
    res1 = P1[:, 2] - np.column_stack([P1[:, 0], P1[:, 1], np.ones(len(P1))]) @ abc1
    # parameter uncertainty: covariance of the (prior-regularized) problem, scaled by the measured misfit
    dof = max(1, len(poses) - n - 3)
    s2 = max(float(np.sum((res1 * 1000 / NOISE_MM) ** 2)) / dof, 0.25)
    try:
        cov = np.linalg.inv(sol.jac.T @ sol.jac) * s2
        std = {j: float(math.sqrt(max(cov[i, i], 0.0))) for i, j in enumerate(joints)}
    except np.linalg.LinAlgError:
        std = dict.fromkeys(joints, float("inf"))

    rms0 = float(np.sqrt(np.mean(res0 ** 2))) * 1000
    rms1 = float(np.sqrt(np.mean(res1 ** 2))) * 1000
    problems = []
    for j in joints:
        if std[j] > MAX_STD_DEG:
            problems.append(f"{j} isn't well determined (±{std[j]:.2f}°) — add touches further apart (near/far)")
        if abs(delta[j]) > MAX_OFFSET_DEG:
            problems.append(f"{j} would change {delta[j]:+.1f}° — too much; check the touches (pressing in? the tray?)")
    if rms1 > rms0 - 0.1:
        problems.append(f"it doesn't make the touches flatter ({rms0:.1f} → {rms1:.1f} mm RMS) — nothing to fix")
    base = dict(model.offsets_deg)
    total = {j: round(base.get(j, 0.0) + delta.get(j, 0.0), 4) for j in sorted({*base, *delta})}
    return {
        "joints": list(joints),
        "delta_deg": {j: round(v, 3) for j, v in delta.items()},
        "std_deg": {j: round(v, 3) for j, v in std.items()},
        "current_deg": {j: round(v, 4) for j, v in base.items()},
        "total_deg": total,
        "rms_before_mm": round(rms0, 2),
        "rms_after_mm": round(rms1, 2),
        "tilt_before_deg": round(_tilt_deg(*abc0[:2]), 2),
        "tilt_after_deg": round(_tilt_deg(*abc1[:2]), 2),
        "note": "",
        "points": [{"r_mm": round(float(ri) * 1000), "az_deg": round(float(ai)), "before_mm": round(float(e0) * 1000, 1),
                    "after_mm": round(float(e1) * 1000, 1)}
                   for ri, ai, e0, e1 in sorted(zip(r, az, res0, res1), key=lambda t: t[0])],
        "ok": not problems,
        "problems": problems,
    }
