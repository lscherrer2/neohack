"""Limits for LLM-driven control: how far one command may move, and where the tip may go.

All positions are meters in the robot base frame: x forward, y left, z up, origin at the bottom
of the base (roughly table height).
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class RemoteLimits:
    default_step_cm: float = 2.0
    max_step_cm: float = 5.0
    default_angle_deg: float = 10.0
    max_angle_deg: float = 20.0
    tip_speed_mm_s: float = 20.0  # average tip speed (peak ~1.5x)
    turn_speed_deg_s: float = 15.0  # tilt/twist speed

    # safe box for the gripper tip
    min_z_m: float = 0.04  # stay this high above the base bottom / table
    max_z_m: float = 0.35
    min_reach_m: float = 0.12  # distance from the base's turning axis
    max_reach_m: float = 0.40
    max_side_m: float = 0.30  # |y|
    joint_limit: float = 90.0  # stay inside +-this of the -100..100 calibrated range

    def check_tip(self, p: np.ndarray, pan_axis_xy: np.ndarray) -> str | None:
        """None if `p` is inside the safe box, otherwise a human-readable reason."""
        x, y, z = (float(v) for v in p)
        reach = float(np.linalg.norm(np.asarray(p)[:2] - pan_axis_xy))
        if z < self.min_z_m:
            return f"too low: tip would be {z * 100:.1f} cm high (minimum {self.min_z_m * 100:.0f} cm)"
        if z > self.max_z_m:
            return f"too high: tip would be {z * 100:.1f} cm high (maximum {self.max_z_m * 100:.0f} cm)"
        if reach < self.min_reach_m:
            return f"too close to the base: reach {reach * 100:.1f} cm (minimum {self.min_reach_m * 100:.0f} cm)"
        if reach > self.max_reach_m:
            return f"too far out: reach {reach * 100:.1f} cm (maximum {self.max_reach_m * 100:.0f} cm)"
        if abs(y) > self.max_side_m:
            return f"too far to the side: {abs(y) * 100:.1f} cm (maximum {self.max_side_m * 100:.0f} cm)"
        return None

    def check_joints(self, pose: dict[str, float]) -> str | None:
        over = {j: round(v, 1) for j, v in pose.items() if j != "gripper" and abs(v) > self.joint_limit}
        return f"joint near its limit: {over}" if over else None
