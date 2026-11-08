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
from packing.boxes.pack import plan_pack_order
from packing.boxes.packer import TrayPacker
from packing.boxes.store import BoxStore
from packing.config import load_config
from packing.hardware.arm import FakeArm
from packing.motion.program import Grip, OpenGripper
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
        T_pack = box_frame((self.cfg.boxes.pack_spot_mm[0] / 1000, self.cfg.boxes.pack_spot_mm[1] / 1000), 0.0, PAN)
        lengths = {t.sku: t.length_mm / 1000 for t in self.cfg.tray.types}
        drops = [i for i in releases if plan.steps[i].label.startswith("item")]
        self.assertEqual(len(drops), 3)
        for i, sku in zip(drops, o.items):
            T = MODEL.fk(plan.steps[i - 1].pose)  # the drop pose
            mid = T[:3, 3] + T[:3, 2] * lengths[sku] / 2  # the held part is centered over the opening...
            self.assertLess(np.linalg.norm((mid - T_pack[:3, 3])[:2]) * 1000, 3.0)
            end = T_pack[:3, :3].T @ (T[:3, 3] + T[:3, 2] * lengths[sku] - T_pack[:3, 3])
            self.assertGreater(end[2], WALL)  # ...its end above the floor
            self.assertGreater(T[2, 3], T_pack[2, 3] + BOX_H)  # jaws above the rim
        self.assertEqual(plan.steps[-1].label, "back out level along the tab")  # ends releasing the box at output

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
