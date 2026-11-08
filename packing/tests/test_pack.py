"""Packing a whole order (packing/boxes/pack.py + packer.py): box to the pack spot, reserved parts into it, box to
its output spot. Tray off to the LEFT of the arm (as on the real cell from 2026-11-07). Sim only."""

import tempfile
import time
import unittest
from pathlib import Path

import numpy as np
from fastapi.testclient import TestClient
from scipy.spatial.transform import Rotation

from packing.boxes.geometry import BOX_H, BOX_W, WALL, box_frame, obstacles
from packing.boxes.pack import (FIT_MARGIN_MM, MAX_JAW_TILT_DEG, RIM_CLEAR_MM, drop_pitches, part_height_mm,
                                plan_pack_order, span_mm)
from packing.boxes.packer import TrayPacker
from packing.boxes.store import BoxStore
from packing.config import load_config
from packing.hardware.arm import FakeArm
from packing.motion.kinematics import POSITION_TOL_M
from packing.motion.program import Grip, JointMove, OpenGripper
from packing.service.orders import Order, OrderStore
from packing.service.vapi import Catalog
from packing.shop.arm_hub import ArmHub
from packing.shop.server import create_server
from packing.shop.shelf import Shelf
from packing.tests.test_arm_view import MODEL
from packing.tests.test_boxes import tool_in_box
from packing.tests import test_tray
from packing.tests.test_tray import temp_config, tilted_pose
from packing.tray.calibration import calibrate
from packing.tray.pick import PickSettings

PAN = MODEL.pan_xy
HOME = dict.fromkeys(["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"], 0.0)
SUPPLY = (0.316, 0.16)  # >= ~30 cm from the swivel: the level tab grip reaches


def left_tray(cfg, az_deg=60, dist=0.33):
    """Tray facing the arm, rotated az_deg to the left about the pan axis. Saved as the calibration."""
    R, t = tilted_pose(86, dist)
    Rz = Rotation.from_euler("z", az_deg, degrees=True).as_matrix()
    p = np.array([PAN[0], PAN[1], 0.0])
    R2, t2 = Rz @ R, Rz @ (t - p) + p
    cal = calibrate([(pid, xy, R2 @ (np.array([*xy, 0]) * 0.001) + t2) for pid, xy in test_tray.CalibrationTests.REFS])
    cal.save(cfg.tray.calibration_file)
    return cal


def order(oid, slots, skus):
    return Order(oid, "Sam", "", skus, "packing", "test", None, None, None, None, None, "", "", slots)


class PlanTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls._tmp = tempfile.TemporaryDirectory()
        cls.cfg = load_config(temp_config(Path(cls._tmp.name)))
        cls.cal = left_tray(cls.cfg)
        cls.boxes = BoxStore(OrderStore(":memory:"))
        cls.box = cls.boxes.add(box_frame(SUPPLY, 0.0, PAN), tool_in_box(), {}, {})

    @classmethod
    def tearDownClass(cls):
        cls._tmp.cleanup()

    def test_box_in_parts_dropped_inside_box_out(self):
        o = order(1, ["aa_1", "glue_3", "piezo_5"], ["aa", "glue", "piezo"])
        plan = plan_pack_order(MODEL, self.cfg, self.cal, o, self.box, HOME, obstacles(self.cfg, {}))
        self.assertEqual([i["pocket"] for i in plan.items], o.slots)
        grips = [s for s in plan.steps if isinstance(s, Grip)]
        self.assertEqual(len(grips), 2 + 3)  # box twice + three parts
        releases = [i for i, s in enumerate(plan.steps) if isinstance(s, OpenGripper) and "release" in s.label]
        drops = [i for i in releases if plan.steps[i].label.startswith("item")]
        self.assertEqual(len(drops), 3)
        self.assert_side_releases(plan, drops, o.items)
        # ends releasing the box at output, straight up, then folded back to sleep (rest_pose)
        self.assertEqual([st.label for st in plan.steps[-3:]],
                         ["back out level along the tab", "straight up 50 mm", "back to sleep"])
        self.assertEqual(plan.steps[-1].pose, self.cfg.rest())
        # starts by folding to sleep; every free move out of sleep only turns the base (+ wrist roll) first, then
        # reaches out in the arm's plane; parts are carried tray -> sleep -> box
        rest = self.cfg.rest()
        moves = [st for st in plan.steps if isinstance(st, JointMove)]
        self.assertEqual(moves[0].label, "to sleep first")
        self.assertEqual(moves[0].pose, rest)
        planar = ("shoulder_lift", "elbow_flex", "wrist_flex")
        for a, b in zip(moves, moves[1:]):
            if a.pose == rest:
                with self.subTest(after=a.label, move=b.label):
                    self.assertEqual(b.label, "turn at sleep")
                    self.assertEqual([b.pose[j] for j in planar], [rest[j] for j in planar])
            if a.label == "turn at sleep":
                self.assertEqual(b.pose["shoulder_pan"], a.pose["shoulder_pan"])
        sleeps = [i for i, st in enumerate(plan.steps) if st.label == "back to sleep"]
        for i in plan.item_steps:  # holding the part: back to sleep between the pick and the drop
            self.assertTrue(any(i["taken"] <= k < i["end"] for k in sleeps))
        self.assertNotIn("up and away from the box", [st.label for st in plan.steps])  # box in: straight up instead

    def assert_side_releases(self, plan, drops, skus):
        """Every release: jaws closing horizontally, the part's center over the opening, jaws + part above the rim."""
        T_pack = box_frame((self.cfg.boxes.pack_spot_mm[0] / 1000, self.cfg.boxes.pack_spot_mm[1] / 1000), 0.0, PAN)
        types = {t.sku: t for t in self.cfg.tray.types}
        rim = T_pack[2, 3] + BOX_H + RIM_CLEAR_MM / 1000 - POSITION_TOL_M
        for i, sku in zip(drops, skus):
            with self.subTest(step=plan.steps[i].label):
                T = MODEL.fk(plan.steps[i - 1].pose)  # the release pose
                z, x, L = T[:3, 2], T[:3, 0], types[sku].length_mm / 1000
                self.assertLess(abs(x[2]), np.sin(np.radians(MAX_JAW_TILT_DEG)))  # jaw axis level
                self.assertLessEqual(-z[2], np.sin(np.radians(70)))  # level or pitched down, never pointing down
                off = PickSettings.for_type(self.cfg.tray, sku, self.cfg.gripper).fixed_jaw_offset_mm / 1000
                mid = T[:3, 3] + z * L / 2 - x * off  # the held part's center...
                self.assertLess(np.linalg.norm((mid - T_pack[:3, 3])[:2]), POSITION_TOL_M + 1e-4)  # ...over the box
                low = T[2, 3] + min(0.0, z[2]) * L - part_height_mm(types[sku]) / 2000
                self.assertGreater(T[2, 3], rim)  # jaws above the rim
                self.assertGreater(low, rim)  # the whole part too
                self.assertIn("up and away", plan.steps[i + 1].label)  # retreat after letting go

    def test_every_pocket_releases_level_jaws_over_the_opening(self):
        """All 33 pockets in one order (sim calibration, pack spot from cell.yaml)."""
        slots = [p.id for p in self.cfg.tray.layout().pockets]
        skus = [s.rsplit("_", 1)[0] for s in slots]
        plan = plan_pack_order(MODEL, self.cfg, self.cal, order(3, slots, skus), self.box, HOME,
                               obstacles(self.cfg, {}))
        self.assertEqual(len(plan.items), 33)
        drops = [i for i, s in enumerate(plan.steps) if isinstance(s, OpenGripper) and s.label.startswith("item")
                 and "release" in s.label]
        self.assertEqual(len(drops), 33)
        self.assert_side_releases(plan, drops, skus)
        by_sku = {}
        for it in plan.items:
            by_sku.setdefault(it["sku"], []).append(it)
        self.assertTrue(all(it["drop_fits_opening"] for it in plan.items))
        self.assertTrue(all(it["drop_tilt_deg"] < 15 for s in ("aa", "9v", "piezo", "breadboard") for it in by_sku[s]))
        self.assertTrue(all(it["drop_tilt_deg"] > 50 for it in by_sku["glue"]))  # longer than the opening: end first
        for e in plan.item_steps:  # bookkeeping kept intact: each item's drop ends where the next pick starts
            self.assertLess(e["start"], e["taken"])
            self.assertLess(e["taken"], e["end"])
        # ...then back to sleep between items: the next pick starts right after that one step
        self.assertTrue(all(plan.steps[e["end"]].label == "back to sleep" for e in plan.item_steps))
        self.assertEqual([e["start"] for e in plan.item_steps[1:]], [e["end"] + 1 for e in plan.item_steps[:-1]])

    def test_drop_pitches(self):
        self.assertEqual(drop_pitches(50, 15.5)[0], 0.0)  # fits level: level first
        p = drop_pitches(120, 12.5)  # glue: longer than the 74.6 mm opening
        self.assertGreater(p[0], 45)
        self.assertLessEqual(span_mm(120, 12.5, p[0]), (BOX_W - 2 * WALL) * 1000 - 2 * FIT_MARGIN_MM)
        self.assertGreater(span_mm(120, 12.5, p[0] - 5), (BOX_W - 2 * WALL) * 1000 - 2 * FIT_MARGIN_MM)
        self.assertIn(0.0, p)  # level stays as the fallback

    def test_needs_every_unit_reserved(self):
        with self.assertRaises(ValueError):
            plan_pack_order(MODEL, self.cfg, self.cal, order(2, ["aa_1", None], ["aa", "aa"]), self.box, HOME)


class PackerTests(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.cfg = load_config(temp_config(Path(self._tmp.name)))
        left_tray(self.cfg)
        self.shelf = Shelf(self.cfg)
        for p in ("aa_1", "glue_3"):
            self.shelf.set_slot(p, True)
        self.store = OrderStore(":memory:")
        self.boxes = BoxStore(self.store)
        self.box = self.boxes.add(box_frame(SUPPLY, 0.0, PAN), tool_in_box(), {}, {})
        self.arm = FakeArm(gain=0.9)
        self.arm.gripper_block = 5.0  # something between the jaws at every grip (tab, parts)
        self.hub = ArmHub(lambda: self.arm, "sim", lambda: MODEL, poll_s=0.01)
        self.hub.connect()
        self.catalog = Catalog(self.cfg.catalog, 10, available=self.shelf.available, stock=self.shelf.stock)

    def tearDown(self):
        self.hub.mode = "idle"
        self.hub.close()
        self._tmp.cleanup()

    def test_dry_run_endpoint_assigns_nothing(self):
        o, _ = self.store.create("", ["aa", "glue"], "Sam", stock=self.shelf.stock)
        http = TestClient(create_server(self.store, self.catalog, shelf=self.shelf, hub=self.hub))
        r = http.post(f"/api/orders/{o.id}/pack-plan")
        self.assertEqual(r.status_code, 200, r.text)
        self.assertEqual([i["pocket"] for i in r.json()["plan"]["items"]], ["aa_1", "glue_3"])
        self.assertEqual(self.boxes.get(self.box.id).state, "available")  # dry run: no box taken
        self.boxes.remove(self.box.id)
        r = http.post(f"/api/orders/{o.id}/pack-plan")
        self.assertEqual(r.status_code, 422)
        self.assertIn("no empty box", r.json()["detail"])

    def test_pack_runs_on_the_arm_and_books_everything(self):
        o, _ = self.store.create("", ["aa", "glue"], "Sam", stock=self.shelf.stock)
        events = []
        t0 = time.monotonic()
        box_name = TrayPacker(self.hub, self.cfg, self.boxes, self.shelf, lambda k, t, **d: events.append(t),
                              sleep=lambda s: None).pack(o)
        self.assertLess(time.monotonic() - t0, 120)
        self.assertEqual(box_name, f"BOX-{self.box.id}")
        b = self.boxes.get(self.box.id)
        self.assertEqual((b.state, b.location, b.order_id), ("packed", "output", o.id))
        self.assertEqual(self.shelf.full_slots() & {"aa_1", "glue_3"}, set())  # the reserved units are used
        self.assertTrue(any("packed into box" in e for e in events))
        self.assertEqual(self.hub.mode, "idle")


if __name__ == "__main__":
    unittest.main()
