"""Sim-only Motion tests (ported from the old Cell tests): following error, sag compensation, feed-forwards."""

import tempfile
import unittest
from pathlib import Path

from packing.cell.inventory import Inventory
from packing.config import load_config
from packing.hardware.arm import FakeArm
from packing.joints import BODY
from packing.motion.controller import Motion, MotionError
from packing.poses import PoseStore

EXAMPLE = Path(__file__).resolve().parents[1] / "cell.example.yaml"
TARGET = {"shoulder_pan": 30.0, "shoulder_lift": -20.0, "elbow_flex": 40.0, "wrist_flex": 0.0, "wrist_roll": 0.0}


class MotionTests(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.tmp = Path(self._tmp.name)

    def tearDown(self):
        self._tmp.cleanup()

    def make(self, gravity=None):
        cfg = load_config(EXAMPLE)
        cfg.poses_file = self.tmp / "poses.json"
        arm = FakeArm(gain=0.6, closed=cfg.gripper.closed, opened=cfg.gripper.open)
        motion = Motion(arm, PoseStore(cfg.poses_file), cfg.motion, cfg.gripper, sleep=lambda s: None,
                        log=lambda s: None, gravity=gravity)
        return cfg, arm, motion

    def test_following_error_stops(self):
        cfg, arm, motion = self.make()
        arm.gain = 0.0  # joints never move = blocked
        with self.assertRaises(MotionError):
            motion.move_to(TARGET)

    def test_sag_compensation_reaches_target(self):
        cfg, arm, motion = self.make()
        arm.sag["shoulder_lift"] = 8.0  # gravity: joint settles 8 units short
        target = {"shoulder_pan": 0.0, "shoulder_lift": 10.0, "elbow_flex": 0.0, "wrist_flex": 0.0, "wrist_roll": 0.0}
        motion.move_to(target)
        self.assertLess(abs(arm.read()["shoulder_lift"] - 10.0), cfg.motion.settle_tol)
        cfg.motion.sag_gain = 0.0
        motion.hold()
        with self.assertRaises(MotionError):  # without compensation it never arrives
            motion.move_to({**target, "shoulder_lift": -10.0})

    def test_sag_bias_frozen_during_moves(self):
        """Lag while moving must not be learned as sag (it made joints overshoot unevenly)."""
        cfg, arm, motion = self.make()
        arm.gain = 0.2  # slow servos: every joint lags its moving command
        seen = []
        real_tick = motion._tick

        def spy(cmd, check_joints, learn=True):
            out = real_tick(cmd, check_joints, learn)
            seen.append((learn, dict(motion.bias)))
            return out

        motion._tick = spy
        motion.move_to(TARGET)
        moving = [bias for learn, bias in seen if not learn]
        self.assertTrue(moving)
        self.assertTrue(all(b == moving[0] for b in moving))  # unchanged for the whole trajectory
        self.assertTrue(any(learn for learn, _ in seen))  # still learns while settling

    def test_velocity_lead_cuts_tracking_lag(self):
        """Feed-forward aims ahead of a moving command so lagging servos stay on schedule (and in step)."""
        worst = {}
        for lead in (0.0, 0.1):
            cfg, arm, motion = self.make()
            arm.gain = 0.2  # first-order lag, time constant ~0.1 s at 50 fps
            cfg.motion.lead_s = lead
            motion.trace = []
            motion.move_to(TARGET)
            worst[lead] = max(abs(c[j] - a[j]) for _, c, a in motion.trace for j in TARGET)
            self.assertLess(abs(arm.read()["elbow_flex"] - 40.0), cfg.motion.settle_tol + 0.5)  # still lands on target
        self.assertLess(worst[0.1], worst[0.0] / 2)

    def test_gravity_feedforward_cancels_sag_during_moves(self):
        """A model-predicted sag is pre-added to every command, so the joint is on target mid-move too."""
        sag = lambda pose: {j: (2.0 if j == "elbow_flex" else 0.0) for j in BODY}  # noqa: E731
        cfg, arm, motion = self.make(gravity=sag)
        arm.sag["elbow_flex"] = 2.0  # gravity: elbow settles 2 units short of whatever is sent
        cfg.motion.sag_gain = 0.0  # the learning loop can't help: feed-forward alone must do it
        target = {"shoulder_pan": 0.0, "shoulder_lift": 0.0, "elbow_flex": 30.0, "wrist_flex": 0.0, "wrist_roll": 0.0}
        motion.move_to(target)
        for _ in range(10):  # keep holding: converges onto the target, not 2 units short
            actual = motion._tick(motion.cmd, BODY)
        self.assertLess(abs(actual["elbow_flex"] - 30.0), 0.1)

        arm.target = arm.read()  # same thing without feed-forward: never arrives
        plain = Motion(arm, motion.poses, cfg.motion, cfg.gripper, sleep=lambda s: None, log=lambda s: None)
        with self.assertRaises(MotionError):
            plain.move_to({**target, "elbow_flex": 0.0})


class InventoryTests(unittest.TestCase):
    def test_restock_parts_and_persist(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "inv.db"
            inv = Inventory(path)
            self.assertEqual(inv.restock({"aa_1": "aa", "aa_2": "aa", "glue_1": "glue"}), 3)
            self.assertEqual(inv.restock({"aa_1": "aa"}), 0)  # already full
            inv.set_part(inv.available_part("aa_1").id, "packed")
            again = Inventory(path)
            self.assertEqual(len(again.parts("packed")), 1)
            self.assertEqual(again.full_slots(), {"aa_2", "glue_1"})


if __name__ == "__main__":
    unittest.main()
