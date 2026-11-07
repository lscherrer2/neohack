"""Joint helpers and the jog key map."""

import unittest

from packing.hardware.keyboard import JOG_KEYS
from packing.joints import BODY, JOINTS, clamp_joint


class JointsTest(unittest.TestCase):
    def test_body_joints_clamp_to_plus_minus_100(self):
        self.assertEqual(clamp_joint("elbow_flex", 150.0), 100.0)
        self.assertEqual(clamp_joint("elbow_flex", -150.0), -100.0)
        self.assertEqual(clamp_joint("elbow_flex", 12.5), 12.5)

    def test_gripper_clamps_to_0_100(self):
        self.assertEqual(clamp_joint("gripper", -5.0), 0.0)
        self.assertEqual(clamp_joint("gripper", 120.0), 100.0)

    def test_every_joint_has_one_key_each_way(self):
        for joint in JOINTS:
            directions = sorted(d for j, d in JOG_KEYS.values() if j == joint)
            self.assertEqual(directions, [-1, 1], joint)
        self.assertEqual(JOINTS[: len(BODY)], BODY)


if __name__ == "__main__":
    unittest.main()
