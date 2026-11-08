"""Desk calibration: the desk's real plane in the robot frame, from 3+ touched points.

The planner assumes "up" is the arm base's z and the desk flat. A slightly tilted base or an uneven desk makes the
real desk height differ from spot to spot (1° over 30 cm ≈ 5 mm). With the motors off, touch the closed jaw tip
to the desk at 3+ spread-out spots (left, middle, right); a least-squares plane z = a·x + b·y + c is fitted.

Box spots then take their height from the plane RELATIVE to the box's calibrated spot:
    bottom(at xy) = bottom(calibrated) + plane(xy) − plane(calibrated xy)
so only height differences matter — a constant offset cancels out.

Each touch records the closed JAW TIP (`touch_point`: the tool point + JAW_TIP_M along the tool axis), so the plane
is the desk surface itself. The plane is also used absolutely, as the floor: the box capture check (box bottom
near the desk), the tray stand (down to the desk under the tray) and `check_desk_clear` (no arm point / jaw tip /
carried box corner below the desk). Without a desk calibration none of that changes (flat desk at z = 0, no desk
check).
"""

from __future__ import annotations

import json
import math
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path

import numpy as np

MAX_RMS_MM = 3.0  # fit refused above this
MIN_AREA_M2 = 0.004  # touched points must span at least this much desk (not all in a line / one spot)
JAW_TIP_M = 0.006  # the closed jaw tips stick out this far past the tool point along the tool axis (meshes, ~5-6 mm)
TIP_CLEAR_M = 0.0  # planned jaw tip never below the desk surface (the plane is good to ~1-3 mm; picks go close)
LINK_CLEAR_M = 0.015  # arm center line (shoulder .. wrist) kept this far above the desk (links are ~3 cm thick)
BOX_CLEAR_M = 0.005  # carried box corners kept this far above the desk


class DeskError(ValueError):
    """Bad desk calibration (message shown to the user)."""


@dataclass
class DeskPlane:
    a: float
    b: float
    c: float  # z = a·x + b·y + c (robot frame, m)
    points: list[list[float]] = field(default_factory=list)  # touched points, m
    rms_mm: float = 0.0
    created: str = ""
    poses: list[dict] = field(default_factory=list)  # body joint pose of each touch (re-derive / joint offset fit)

    def z_at(self, x: float, y: float) -> float:
        return self.a * x + self.b * y + self.c

    @property
    def tilt_deg(self) -> float:
        return math.degrees(math.atan(math.hypot(self.a, self.b)))

    def to_dict(self) -> dict:
        return {**asdict(self), "tilt_deg": round(self.tilt_deg, 2)}

    def save(self, path: str | Path) -> Path:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(asdict(self), indent=2))
        return path

    @classmethod
    def load(cls, path: str | Path) -> DeskPlane | None:
        path = Path(path)
        return cls(**json.loads(path.read_text())) if path.exists() else None


def fit_plane(points: list, poses: list[dict] | None = None) -> DeskPlane:
    """Least-squares plane through 3+ touched points (m). DeskError if too few, too bunched up or too uneven.
    `poses`: the joint pose of each touch, kept with the plane."""
    P = np.asarray(points, float)
    if len(P) < 3:
        raise DeskError(f"touch at least 3 spots on the desk ({len(P)} so far)")
    from scipy.spatial import ConvexHull

    try:
        area = float(ConvexHull(P[:, :2]).volume)  # 2D hull "volume" = the area the points enclose
    except Exception:  # noqa: BLE001 — all in a line / one spot
        area = 0.0
    if area < MIN_AREA_M2:
        raise DeskError("the points are too close together or in a line — touch spots far apart (left, middle, right)")
    A = np.column_stack([P[:, 0], P[:, 1], np.ones(len(P))])
    (a, b, c), *_ = np.linalg.lstsq(A, P[:, 2], rcond=None)
    rms = float(np.sqrt(np.mean((A @ [a, b, c] - P[:, 2]) ** 2))) * 1000
    if rms > MAX_RMS_MM:
        raise DeskError(f"the points don't lie on a plane (off by {rms:.1f} mm RMS, max {MAX_RMS_MM}) — re-touch them")
    return DeskPlane(float(a), float(b), float(c), [[round(float(v), 5) for v in p] for p in P], round(rms, 2),
                     time.strftime("%Y-%m-%dT%H:%M:%S"), [dict(q) for q in poses or []])


def touch_point(T_tool: np.ndarray) -> np.ndarray:
    """Where the closed jaw tips are for tool pose T_tool (4x4): what touches the desk."""
    T_tool = np.asarray(T_tool, float)
    return T_tool[:3, 3] + T_tool[:3, 2] * JAW_TIP_M


def corrections_mm(plane: DeskPlane | None, ref_xy, spots: dict) -> dict:
    """Height change (mm) of each named spot {name: xy} vs the calibrated spot ref_xy; {} without a plane."""
    if plane is None:
        return {}
    return {k: round((plane.z_at(*xy) - plane.z_at(*ref_xy)) * 1000, 1) for k, xy in spots.items()}


def _poses_of(step, prev: dict | None, samples: int) -> list[dict]:
    from ..motion.program import JointMove, LineMove

    if isinstance(step, LineMove) and step.path:
        return step.path[:: max(1, len(step.path) // samples)] + [step.path[-1]]
    if isinstance(step, JointMove) and prev is not None:
        return [{j: prev[j] + (step.pose[j] - prev[j]) * k / samples for j in step.pose} for k in range(samples + 1)]
    return []


def desk_point_error(model, q: dict, plane: DeskPlane) -> tuple[str, np.ndarray] | None:
    """(what, point) if the arm at pose q comes too close to the desk: jaw tip below the surface + TIP_CLEAR_M, or
    the arm's center line (shoulder .. wrist, not the jaws) below the surface + LINK_CLEAR_M. None if clear."""
    frames = model.link_frames(q)
    tip = touch_point(frames["gripper_frame_link"])
    if tip[2] < plane.z_at(tip[0], tip[1]) + TIP_CLEAR_M:
        return "the jaw tips would touch the desk", tip
    chain = [frames[n][:3, 3] for n in ("upper_arm_link", "lower_arm_link", "wrist_link", "gripper_link")]
    for pt in chain + [(chain[i] + chain[i + 1]) / 2 for i in range(len(chain) - 1)]:
        if pt[2] < plane.z_at(pt[0], pt[1]) + LINK_CLEAR_M:
            return "the arm would hit the desk", pt
    return None


def check_desk_clear(model, start: dict, steps: list, plane: DeskPlane | None, label: str, samples: int = 10) -> None:
    """Every pose of a program (joint moves along their interpolation, lines / paths along their poses) keeps the
    jaw tips and the arm above the desk plane. No-op without a desk calibration. Raises PathError."""
    if plane is None:
        return
    from ..motion.kinematics import PathError

    prev = dict(start)
    for k, st in enumerate(steps):
        for q in _poses_of(st, prev, samples):
            if hit := desk_point_error(model, q, plane):
                raise PathError(f"{label}: '{st.label}' — {hit[0]}", hit[1], list(steps[: k + 1]), start)
        if getattr(st, "path", None):
            prev = st.path[-1]
        elif isinstance(getattr(st, "pose", None), dict):
            prev = st.pose


def box_below_desk(T_box: np.ndarray, plane: DeskPlane | None) -> float:
    """How far (m) the lowest corner of a box at T_box is below the desk + BOX_CLEAR_M (≤ 0 = clear)."""
    if plane is None:
        return 0.0
    from .geometry import block_corners, box_block

    c = block_corners(box_block(T_box))
    return float(max(plane.z_at(x, y) + BOX_CLEAR_M - z for x, y, z in c))


def height_at(plane: DeskPlane | None, ref_xy, xy, ref_z: float) -> float:
    """Box bottom at xy, given its calibrated bottom ref_z at ref_xy (unchanged without a desk plane)."""
    if plane is None:
        return ref_z
    return ref_z + plane.z_at(*xy) - plane.z_at(*ref_xy)
