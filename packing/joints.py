"""Joint names and value ranges shared by every module.

Body joints use calibrated units in -100..100 (0 = calibration mid pose); the gripper is 0..100.
"""

from __future__ import annotations

BODY = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
JOINTS = [*BODY, "gripper"]

BODY_MIN, BODY_MAX = -100.0, 100.0
GRIPPER_MIN, GRIPPER_MAX = 0.0, 100.0


def clamp_joint(joint: str, value: float) -> float:
    """Limit `value` to the valid range of `joint`."""
    low = GRIPPER_MIN if joint == "gripper" else BODY_MIN
    return max(low, min(BODY_MAX, value))
