"""Tray calibration: where the tray sits relative to the robot.

The user hand-guides the gripper tip to a few reference points (tray coordinates known from the DXF)
and we record the tip position (robot base frame, from FK). The fit gives rotation R and translation t:
p_robot = R @ p_tray + t, with tray points in meters and z = 0 on the face. It averages rather than trusting
any single touch: the tray face = least-squares plane through all touches, the tray's position = their
centroid, and its in-plane turn = the best 2D rotation of the DXF points onto the touches (projected into
that plane). So a corner touched a few mm off only nudges the average.

Tray normal = R[:, 2] (out of the face). Tray frame conventions: see `layout.py`.
"""

from __future__ import annotations

import json
import math
from dataclasses import asdict, dataclass, field
from datetime import datetime
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

MIN_POINTS = 3
MAX_RMS_MM = 20.0  # refuse only clearly wrong touches (wrong / swapped corner: tens of mm)
WARN_RMS_MM = 3.0  # above this the fit is still used, with a warning
MM = 0.001


class CalibrationError(ValueError):
    """The reference points can't give a trustworthy tray pose."""


def fit_rigid(src: np.ndarray, dst: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Rotation R and translation t with dst ≈ R @ src + t (rows are points, n >= 3; src on the tray face,
    z = 0). Plane + centroid fit: the face normal is the least-squares plane through dst, the centroids are
    matched, and the in-plane rotation is a 2D Procrustes fit of src onto dst projected into that plane."""
    src, dst = np.asarray(src, float), np.asarray(dst, float)
    if src.shape != dst.shape or src.shape[0] < MIN_POINTS:
        raise CalibrationError(f"need at least {MIN_POINTS} matching points")
    cs, cd = src.mean(axis=0), dst.mean(axis=0)
    a, b = src - cs, dst - cd
    sv = np.linalg.svd(a, compute_uv=False)
    if sv[1] < 1e-3 * max(sv[0], 1e-12):
        raise CalibrationError("reference points are (nearly) in a line; pick points spread over the tray")
    # Plane through the touches: normal = direction of least spread. Sign from the full rigid fit, which
    # knows the tray's handedness (which side is the front face).
    _, _, vt = np.linalg.svd(b)
    n = vt[2]
    rough = Rotation.align_vectors(b, a)[0].as_matrix()
    if n @ rough[:, 2] < 0:
        n = -n
    e1 = rough[:, 0] - (rough[:, 0] @ n) * n  # any in-plane basis (e1, e2, n) right-handed
    e1 /= np.linalg.norm(e1)
    e2 = np.cross(n, e1)
    # Best in-plane rotation (2D Procrustes, no reflection) of the tray xy onto the projected touches.
    p = np.column_stack([b @ e1, b @ e2])
    q = a[:, :2]
    theta = math.atan2(float(np.sum(q[:, 0] * p[:, 1] - q[:, 1] * p[:, 0])),
                       float(np.sum(q[:, 0] * p[:, 0] + q[:, 1] * p[:, 1])))
    x = math.cos(theta) * e1 + math.sin(theta) * e2
    R = np.column_stack([x, np.cross(n, x), n])
    return R, cd - R @ cs


@dataclass
class RefPoint:
    pocket: str
    tray_mm: list[float]  # [x, y] in the tray frame
    robot_m: list[float]  # [x, y, z] measured tip position
    residual_mm: float = 0.0


@dataclass
class TrayCalibration:
    R: list[list[float]]
    t: list[float]  # meters
    points: list[RefPoint] = field(default_factory=list)
    rms_mm: float = 0.0
    created: str = ""

    # ---------- geometry ----------
    @property
    def rot(self) -> np.ndarray:
        return np.array(self.R)

    @property
    def normal(self) -> np.ndarray:
        """Unit vector out of the tray face, robot frame."""
        return self.rot[:, 2]

    def to_robot(self, x_mm: float, y_mm: float, z_mm: float = 0.0) -> np.ndarray:
        """Tray-frame point (mm) -> robot base frame (m)."""
        return self.rot @ (np.array([x_mm, y_mm, z_mm]) * MM) + np.array(self.t)

    def dir_to_robot(self, dx: float, dy: float, dz: float = 0.0) -> np.ndarray:
        return self.rot @ np.array([dx, dy, dz], float)

    def matrix(self) -> list[list[float]]:
        """4x4 tray -> robot transform (meters)."""
        T = np.eye(4)
        T[:3, :3], T[:3, 3] = self.rot, self.t
        return T.round(6).tolist()

    # ---------- persistence ----------
    def save(self, path: str | Path) -> Path:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(asdict(self), indent=2))
        return path

    @classmethod
    def load(cls, path: str | Path) -> TrayCalibration | None:
        path = Path(path)
        if not path.exists():
            return None
        raw = json.loads(path.read_text())
        raw["points"] = [RefPoint(**p) for p in raw.get("points", [])]
        return cls(**raw)


def calibrate(points: list[tuple[str, tuple[float, float], np.ndarray]],
              max_rms_mm: float = MAX_RMS_MM) -> TrayCalibration:
    """points: (pocket id, tray (x, y) mm, measured robot tip (x, y, z) m). Raises CalibrationError if
    the fit is degenerate or its RMS error is above `max_rms_mm`."""
    src = np.array([[x * MM, y * MM, 0.0] for _, (x, y), _ in points])
    dst = np.array([np.asarray(q, float) for _, _, q in points])
    R, t = fit_rigid(src, dst)
    res = np.linalg.norm((src @ R.T + t) - dst, axis=1) / MM
    rms = float(np.sqrt(np.mean(res**2)))
    cal = TrayCalibration(
        R=R.round(9).tolist(), t=t.round(9).tolist(),
        points=[RefPoint(pid, [float(x), float(y)], [round(float(v), 6) for v in q], round(float(r), 2))
                for (pid, (x, y), q), r in zip(points, res)],
        rms_mm=round(rms, 2), created=datetime.now().isoformat(timespec="seconds"),
    )
    if rms > max_rms_mm:
        worst = max(cal.points, key=lambda p: p.residual_mm)
        raise CalibrationError(f"fit error {rms:.1f} mm RMS (max {max_rms_mm}); worst point {worst.pocket} "
                               f"off by {worst.residual_mm:.1f} mm — redo that point")
    return cal


# ---------------------------------------------------------------- sanity checks
def tilt_deg(cal: TrayCalibration) -> float:
    """Angle of the tray face from horizontal (0 = flat, facing the ceiling; 90 = vertical)."""
    return math.degrees(math.acos(max(-1.0, min(1.0, float(cal.normal[2])))))


def approach_misalignment_deg(cal: TrayCalibration, x_mm: float, y_mm: float, pan_xy: np.ndarray) -> float:
    """How far the tray normal at a pocket leans out of the arm's vertical plane through that pocket.

    The SO-101 has no tool yaw: the gripper axis always lies in the vertical plane through the pan axis
    and the tip. A pocket off the tray's centerline needs an approach along the tray normal, which is
    out of that plane by this angle — the item is gripped tilted by this much.
    """
    p = cal.to_robot(x_mm, y_mm)
    radial = p[:2] - np.asarray(pan_xy)[:2]
    if np.linalg.norm(radial) < 1e-9:
        return 0.0
    radial /= np.linalg.norm(radial)
    side = np.array([-radial[1], radial[0], 0.0])  # horizontal, perpendicular to the arm's plane
    return math.degrees(math.asin(min(1.0, abs(float(cal.normal @ side)))))


def checks(cal: TrayCalibration, center_mm: tuple[float, float], pan_xy: np.ndarray) -> list[str]:
    """Human-readable warnings for a calibration that looks physically wrong."""
    warn: list[str] = []
    n = cal.normal
    if n[2] < 0:
        warn.append("tray face points down — were the reference points taken on the front face, in order?")
    c = cal.to_robot(*center_mm)
    to_base = np.asarray(pan_xy)[:2] - c[:2]
    horiz = n[:2]
    if np.linalg.norm(horiz) > 0.05 and np.linalg.norm(to_base) > 1e-6:
        facing = math.degrees(math.acos(float(np.clip(horiz @ to_base / np.linalg.norm(horiz) / np.linalg.norm(to_base), -1, 1))))
        if facing > 15:
            warn.append(f"tray doesn't face the arm's swivel: off by {facing:.0f}°")
    x_axis = cal.dir_to_robot(1, 0, 0)
    roll = math.degrees(math.asin(float(np.clip(x_axis[2], -1, 1))))
    if abs(roll) > 5:
        warn.append(f"tray's left-right edge isn't level: {roll:+.0f}°")
    return warn
