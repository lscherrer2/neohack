"""Sim-only tests: python -m unittest discover -s packing/tests -t .   (from repo root)"""

import tempfile
import unittest
from pathlib import Path

from packing.cell.inventory import GRIPPER, Inventory
from packing.cell.procedures import Cell, CellError
from packing.config import load_config
from packing.hardware.arm import FakeArm
from packing.joints import BODY
from packing.motion.controller import GraspError, Motion, MotionError
from packing.poses import PoseStore

EXAMPLE = Path(__file__).resolve().parents[1] / "cell.example.yaml"


def make_cell(tmp: Path, block: float | None = 15.0):
    cfg = load_config(EXAMPLE)
    cfg.poses_file, cfg.inventory_db = tmp / "poses.json", tmp / "inv.db"
    poses = PoseStore(cfg.poses_file)
    for i, name in enumerate(cfg.required_poses()):  # fake taught poses
        if name != "home":
            poses.set(name, {j: (i * 7 + k * 3) % 80 - 40 for k, j in enumerate(BODY)})
    arm = FakeArm(gain=0.6, closed=cfg.gripper.closed, opened=cfg.gripper.open)
    arm.gripper_block = block
    inv = Inventory(cfg.inventory_db)
    m = Motion(arm, poses, cfg.motion, cfg.gripper, sleep=lambda s: None, log=lambda s: None)
    return cfg, arm, inv, Cell(cfg, m, inv, log=lambda s: None)


class PackingTests(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.tmp = Path(self._tmp.name)

    def tearDown(self):
        self._tmp.cleanup()

    def test_full_package_cycle(self):
        cfg, arm, inv, cell = make_cell(self.tmp)
        inv.restock(cfg.part_slots, cfg.box_supply)
        bid = cell.package_box()
        box = inv.box(bid)
        self.assertEqual((box.status, box.location), ("packed", "box_out_1"))
        contents = inv.contents(bid)
        self.assertEqual(sorted(contents), [0, 1])
        self.assertEqual([contents[0].sku, contents[1].sku], ["A", "B"])
        self.assertEqual(inv.in_gripper(), [])
        self.assertEqual(len(inv.parts("available")), 2)

    def test_second_box_then_out_of_stock_and_resume(self):
        cfg, arm, inv, cell = make_cell(self.tmp)
        inv.restock(cfg.part_slots, cfg.box_supply)
        cell.package_box()
        cell.package_box()  # uses the remaining A2/B2
        with self.assertRaises(CellError):
            cell.package_box()  # no empty boxes left
        inv.restock(cfg.part_slots, [cfg.box_supply[0]])  # refill parts + one box
        cfg.part_slots = {"part_A1": "A"}  # pretend only A slots exist now -> B slot can't be filled
        with self.assertRaises(CellError):
            cell.package_box()
        station_box = inv.box_at(cfg.pack_station)
        self.assertEqual(station_box.status, "packing")
        self.assertEqual(len(inv.contents(station_box.id)), 1)

    def test_missed_grasp_marks_missing(self):
        cfg, arm, inv, cell = make_cell(self.tmp, block=None)  # gripper closes on air
        inv.restock(cfg.part_slots, cfg.box_supply)
        with self.assertRaises(GraspError):
            cell.package_box()  # can't even grab the box
        self.assertEqual(inv.box("BOX-0001").location, "box_in_1")
        self.assertEqual(inv.in_gripper(), [])

    def test_missed_part_falls_back_to_next_part(self):
        cfg, arm, inv, cell = make_cell(self.tmp)
        inv.restock(cfg.part_slots, cfg.box_supply)
        real_grip, calls = cell.motion.grip, []

        def flaky(min_width):
            calls.append(min_width)
            if len(calls) == 2:  # call 1 = box, call 2 = first part (A1) -> miss
                raise GraspError("simulated miss")
            return real_grip(min_width)

        cell.motion.grip = flaky
        bid = cell.package_box()
        missing = inv.parts("missing")
        self.assertEqual([p.location for p in missing], ["part_A1"])
        self.assertEqual(inv.contents(bid)[0].location, bid)
        self.assertEqual(inv.contents(bid)[0].sku, "A")

    def test_refuses_to_run_with_item_in_gripper(self):
        cfg, arm, inv, cell = make_cell(self.tmp)
        inv.restock(cfg.part_slots, cfg.box_supply)
        inv.move_box("BOX-0001", GRIPPER)
        with self.assertRaises(CellError):
            cell.package_box()

    def test_following_error_stops(self):
        cfg, arm, inv, cell = make_cell(self.tmp)
        arm.gain = 0.0  # joints never move = blocked
        with self.assertRaises(MotionError):
            cell.motion.move_to("box_in_1.above")

    def test_sag_compensation_reaches_target(self):
        cfg, arm, inv, cell = make_cell(self.tmp)
        arm.sag["shoulder_lift"] = 8.0  # gravity: joint settles 8 units short
        target = {"shoulder_pan": 0.0, "shoulder_lift": 10.0, "elbow_flex": 0.0, "wrist_flex": 0.0, "wrist_roll": 0.0}
        cell.motion.move_to(target)
        self.assertLess(abs(arm.read()["shoulder_lift"] - 10.0), cfg.motion.settle_tol)
        cfg.motion.sag_gain = 0.0
        cell.motion.hold()
        with self.assertRaises(MotionError):  # without compensation it never arrives
            cell.motion.move_to({**target, "shoulder_lift": -10.0})

    def test_sag_bias_frozen_during_moves(self):
        """Lag while moving must not be learned as sag (it made joints overshoot unevenly)."""
        cfg, arm, inv, cell = make_cell(self.tmp)
        arm.gain = 0.2  # slow servos: every joint lags its moving command
        seen = []
        real_tick = cell.motion._tick

        def spy(cmd, check_joints, learn=True):
            out = real_tick(cmd, check_joints, learn)
            seen.append((learn, dict(cell.motion.bias)))
            return out

        cell.motion._tick = spy
        cell.motion.move_to({"shoulder_pan": 30.0, "shoulder_lift": -20.0, "elbow_flex": 40.0,
                             "wrist_flex": 0.0, "wrist_roll": 0.0})
        moving = [bias for learn, bias in seen if not learn]
        self.assertTrue(moving)
        self.assertTrue(all(b == moving[0] for b in moving))  # unchanged for the whole trajectory
        self.assertTrue(any(learn for learn, _ in seen))  # still learns while settling

    def test_velocity_lead_cuts_tracking_lag(self):
        """Feed-forward aims ahead of a moving command so lagging servos stay on schedule (and in step)."""
        target = {"shoulder_pan": 30.0, "shoulder_lift": -20.0, "elbow_flex": 40.0, "wrist_flex": 0.0, "wrist_roll": 0.0}
        worst = {}
        for lead in (0.0, 0.1):
            cfg, arm, inv, cell = make_cell(self.tmp)
            arm.gain = 0.2  # first-order lag, time constant ~0.1 s at 50 fps
            cfg.motion.lead_s = lead
            cell.motion.trace = []
            cell.motion.move_to(target)
            worst[lead] = max(abs(c[j] - a[j]) for _, c, a in cell.motion.trace for j in target)
            self.assertLess(abs(arm.read()["elbow_flex"] - 40.0), cfg.motion.settle_tol + 0.5)  # still lands on target
        self.assertLess(worst[0.1], worst[0.0] / 2)

    def test_gravity_feedforward_cancels_sag_during_moves(self):
        """A model-predicted sag is pre-added to every command, so the joint is on target mid-move too."""
        cfg, arm, inv, cell = make_cell(self.tmp)
        arm.sag["elbow_flex"] = 2.0  # gravity: elbow settles 2 units short of whatever is sent
        cfg.motion.sag_gain = 0.0  # the learning loop can't help: feed-forward alone must do it
        target = {"shoulder_pan": 0.0, "shoulder_lift": 0.0, "elbow_flex": 30.0, "wrist_flex": 0.0, "wrist_roll": 0.0}
        sag = lambda pose: {j: (2.0 if j == "elbow_flex" else 0.0) for j in BODY}  # noqa: E731
        motion = Motion(arm, cell.motion.poses, cfg.motion, cfg.gripper, sleep=lambda s: None, log=lambda s: None,
                        gravity=sag)
        motion.move_to(target)
        for _ in range(10):  # keep holding: converges onto the target, not 2 units short
            actual = motion._tick(motion.cmd, BODY)
        self.assertLess(abs(actual["elbow_flex"] - 30.0), 0.1)

        arm.target = arm.read()  # same thing without feed-forward: never arrives
        plain = Motion(arm, cell.motion.poses, cfg.motion, cfg.gripper, sleep=lambda s: None, log=lambda s: None)
        with self.assertRaises(MotionError):
            plain.move_to({**target, "elbow_flex": 0.0})

    def test_inventory_persists(self):
        cfg, arm, inv, cell = make_cell(self.tmp)
        inv.restock(cfg.part_slots, cfg.box_supply)
        cell.package_box()
        again = Inventory(cfg.inventory_db)
        self.assertEqual(len(again.parts("packed")), 2)


if __name__ == "__main__":
    unittest.main()
