"""Desk calibration: the desk's real plane in the robot frame, from 3+ touched points.

The planner assumes "up" is the arm base's z and the desk flat. A slightly tilted base or an uneven desk makes the
real desk height differ from spot to spot (1° over 30 cm ≈ 5 mm). With the motors off, touch the closed jaw tip
to the desk at 3+ spread-out spots (left, middle, right); a least-squares plane z = a·x + b·y + c is fitted.

Box spots then take their height from the plane RELATIVE to the box's calibrated spot:
    bottom(at xy) = bottom(calibrated) + plane(xy) − plane(calibrated xy)
so only height differences matter — a constant offset (e.g. the jaw tips sticking out past the tool point)
cancels out.
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


def fit_plane(points: list) -> DeskPlane:
    """Least-squares plane through 3+ touched points (m). DeskError if too few, too bunched up or too uneven."""
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
                     time.strftime("%Y-%m-%dT%H:%M:%S"))


def height_at(plane: DeskPlane | None, ref_xy, xy, ref_z: float) -> float:
    """Box bottom at xy, given its calibrated bottom ref_z at ref_xy (unchanged without a desk plane)."""
    if plane is None:
        return ref_z
    return ref_z + plane.z_at(*xy) - plane.z_at(*ref_xy)
