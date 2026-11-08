"""Plan a "go to" preview over a calibrated tray point: tip a short stand-off out along the tray normal, tool
pointing into the tray as closely as the 5-DOF arm allows.

Two stages, both fully planned and validated before anything moves:
  1. joint-space move to an approach point `approach_mm` out along the normal (aligned orientation),
  2. straight line (IK, constant orientation) in to `standoff_mm` from the point.

Orientation: the tool's approach axis is gripper_frame_link +z. The SO-101 has no wrist yaw, so the approach
axis always lies in the vertical plane through the pan axis and the tip; the best achievable direction is the
tray's inward normal projected into that plane. The leftover angle is reported (`tilt_deg`).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from ..joints import BODY
from ..motion.kinematics import ArmModel, PathError

INCH_MM = 25.4
MAX_IK_ERR_M = 0.002
JOINT_LIMIT = 100.0
LINE_SPEED_MM_S = 20.0  # straight line in, slow


@dataclass
class GotoPlan:
    point: str
    point_robot_m: list[float]
    approach_pose: dict[str, float]  # joint pose at the approach point
    path: list[dict[str, float]]  # straight line approach -> stand-off (joint poses)
    standoff_mm: float
    approach_mm: float
    tilt_deg: float  # angle between the tool axis and the tray's inward normal (5-DOF limit)
    travel_units: float  # largest joint change of the free move (calibrated units)
    line_mm: float

    @property
    def label(self) -> str:
        return f"go to {self.point} (+{self.standoff_mm:.1f} mm along the normal)"

    @property
    def steps(self) -> list:
        """As a motion program: slow free move to the approach point, then the straight line in."""
        from ..motion.program import JointMove, LineMove

        return [JointMove(self.approach_pose, None, f"free move to {self.approach_mm:.0f} mm out"),
                LineMove(self.path, self.line_mm, LINE_SPEED_MM_S, f"line in to {self.standoff_mm:.1f} mm")]

    def summary(self) -> dict:
        return {"point": self.point, "point_robot_m": self.point_robot_m, "standoff_mm": self.standoff_mm,
                "approach_mm": self.approach_mm, "tilt_deg": round(self.tilt_deg, 1),
                "travel_units": round(self.travel_units, 1), "line_mm": round(self.line_mm, 1),
                "approach_pose": {k: round(v, 1) for k, v in self.approach_pose.items()},
                "end_pose": {k: round(v, 1) for k, v in self.path[-1].items()}}


def aligned_target(model: ArmModel, p: np.ndarray, inward: np.ndarray) -> tuple[np.ndarray, float]:
    """4x4 tool target at point p whose approach axis (+z) is `inward` projected into the arm's vertical plane
    through p. Returns (T, angle in degrees between the achievable axis and `inward`)."""
    az = model.azimuth(p)
    side = np.array([-math.sin(az), math.cos(az), 0.0])  # arm-plane normal (= tool y at wrist roll 0)
    z = inward - side * float(inward @ side)
    if np.linalg.norm(z) < 1e-6:
        raise PathError("the tray normal is perpendicular to the arm's plane — can't point the tool along it")
    z /= np.linalg.norm(z)
    T = np.eye(4)
    T[:3, :3] = np.column_stack([np.cross(side, z), side, z])
    T[:3, 3] = p
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(z @ inward)))))
    return T, tilt


def _solve(model: ArmModel, seeds: list[dict[str, float]], T: np.ndarray) -> tuple[dict[str, float], float]:
    """Best IK solution over several seeds: within joint limits first, then smallest position error."""
    results = []
    for seed in seeds:
        pose, err = model.ik({j: seed[j] for j in BODY}, T, orientation_weight=0.5, iters=150)
        results.append((any(abs(v) > JOINT_LIMIT for v in pose.values()), err, pose))
    over, err, pose = min(results, key=lambda r: (r[0], r[1]))
    return pose, err


def plan_goto(model: ArmModel, point: str, point_m: np.ndarray, normal: np.ndarray, current: dict[str, float],
              standoff_mm: float = INCH_MM / 2, approach_mm: float = 60.0) -> GotoPlan:
    """Plan the preview move from joint pose `current` to `standoff_mm` out along `normal` from `point_m`.
    Raises PathError if any part is unreachable or would exceed joint limits."""
    n = np.asarray(normal, float) / np.linalg.norm(normal)
    c = np.asarray(point_m, float)
    above = c + n * approach_mm / 1000
    near = c + n * standoff_mm / 1000
    T_above, tilt = aligned_target(model, above, -n)
    bent = dict(shoulder_lift=-10, elbow_flex=10, wrist_flex=90, wrist_roll=0)  # reaching down, forward
    seeds = [current, *({"shoulder_pan": float(pan), **bent} for pan in (current["shoulder_pan"], -60, -30, 0, 30, 60))]
    pose, err = _solve(model, seeds, T_above)
    where = f"approach point ({np.round(above * 1000, 1)} mm)"
    if err > MAX_IK_ERR_M:
        raise PathError(f"can't reach the {where}: IK off by {err * 1000:.1f} mm")
    over = {j: round(v, 1) for j, v in pose.items() if abs(v) > JOINT_LIMIT}
    if over:
        raise PathError(f"joint limit at the {where}: {over}")
    path = model.line(pose, near, limit=JOINT_LIMIT)  # keeps the aligned orientation; validates every step
    travel = max(abs(pose[j] - current[j]) for j in BODY)
    z_end = model.fk(path[-1])[:3, 2]  # achieved tool axis (IK may not hit the target orientation exactly)
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(z_end @ -n)))))
    return GotoPlan(point, [round(float(v), 5) for v in c], pose, path, standoff_mm, approach_mm, tilt, travel,
                    float(np.linalg.norm(above - near) * 1000))
