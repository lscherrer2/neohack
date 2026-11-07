"""Straight-line IK tests (skipped if placo / calibration missing)."""

import unittest

import numpy as np

try:
    from packing.motion.kinematics import ArmModel, PathError, load_ranges

    MODEL = ArmModel(load_ranges("my_awesome_follower_arm"))
except Exception as e:  # noqa: BLE001
    MODEL, WHY = None, str(e)

A = {"shoulder_pan": -15.0, "shoulder_lift": 0.0, "elbow_flex": 0.0, "wrist_flex": 10.0, "wrist_roll": 0.0}
B = {**A, "shoulder_pan": 15.0}


def deviation_mm(tips, p0, p1):
    d = (p1 - p0) / np.linalg.norm(p1 - p0)
    return max(np.linalg.norm((t - p0) - np.dot(t - p0, d) * d) for t in tips) * 1000


@unittest.skipIf(MODEL is None, "kinematics unavailable")
class KinematicsTests(unittest.TestCase):
    def test_units_roundtrip(self):
        back = MODEL.to_units(MODEL.to_deg(A))
        for j in A:
            self.assertAlmostEqual(back[j], A[j], places=6)

    def test_urdf_zero_is_the_calibration_pose(self):
        """URDF 0 deg = raw 2047 (where calibration left each joint), NOT the middle of the range.
        Using the range middle put the gripper ~15 deg off level (elbow alone: 8.7 deg)."""
        ranges = load_ranges("my_awesome_follower_arm")
        pose = {}
        for j, (lo, hi) in ranges.items():  # units at which raw == 2047
            pose[j] = (2047 - lo) / (hi - lo) * 200 - 100
        np.testing.assert_allclose(MODEL.to_deg(pose), 0.0, atol=1e-9)

    def test_line_is_straight_and_joint_move_is_not(self):
        p0, p1 = MODEL.tip(B), MODEL.tip(A)
        line = MODEL.line(B, A)
        self.assertLess(deviation_mm([MODEL.tip(p) for p in line], p0, p1), 1.0)
        joint = [{j: B[j] + (A[j] - B[j]) * s for j in A} for s in np.linspace(0, 1, 50)]
        self.assertGreater(deviation_mm([MODEL.tip(p) for p in joint], p0, p1), 5.0)

    def test_unreachable_point_raises(self):
        with self.assertRaises(PathError):
            MODEL.line(A, np.array([2.0, 0.0, 0.2]))  # 2 m away


if __name__ == "__main__":
    unittest.main()
