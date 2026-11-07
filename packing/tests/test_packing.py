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

    def test_inventory_persists(self):
        cfg, arm, inv, cell = make_cell(self.tmp)
        inv.restock(cfg.part_slots, cfg.box_supply)
        cell.package_box()
        again = Inventory(cfg.inventory_db)
        self.assertEqual(len(again.parts("packed")), 2)


if __name__ == "__main__":
    unittest.main()
