"""Desk plane calibration (packing/boxes/desk.py): fit, refusals, box spot heights following a tilted desk, and the
API on a FakeArm. Sim only."""

import tempfile
import unittest
from pathlib import Path

import numpy as np
from fastapi.testclient import TestClient

from packing.boxes.desk import DeskError, DeskPlane, fit_plane, height_at
from packing.boxes.geometry import box_frame
from packing.boxes.move import box_pose
from packing.boxes.store import Box
from packing.config import load_config
from packing.hardware.arm import FakeArm
from packing.service.orders import OrderStore
from packing.service.vapi import Catalog
from packing.shop.arm_hub import ArmHub
from packing.shop.server import create_server
from packing.shop.shelf import Shelf
from packing.tests.test_arm_view import MODEL
from packing.tests.test_boxes import tool_in_box
from packing.tests.test_tray import temp_config

PAN = MODEL.pan_xy


def tilted(points_xy, a=0.02, b=-0.01, c=0.006, noise=0.0, seed=0):
    rng = np.random.default_rng(seed)
    return [[x, y, a * x + b * y + c + rng.normal(0, noise)] for x, y in points_xy]


SPOTS = [(0.30, 0.20), (0.35, 0.0), (0.30, -0.20), (0.20, 0.10)]


class FitTests(unittest.TestCase):
    def test_recovers_tilt(self):
        p = fit_plane(tilted(SPOTS, noise=0.0005))
        self.assertAlmostEqual(p.a, 0.02, delta=0.005)
        self.assertAlmostEqual(p.b, -0.01, delta=0.005)
        self.assertLess(p.rms_mm, 1.0)
        self.assertAlmostEqual(p.tilt_deg, np.degrees(np.arctan(np.hypot(0.02, 0.01))), delta=0.3)

    def test_refuses_too_few_bunched_or_uneven(self):
        with self.assertRaises(DeskError):
            fit_plane(tilted(SPOTS[:2]))
        with self.assertRaises(DeskError):  # all in a line
            fit_plane(tilted([(0.2, 0.0), (0.3, 0.0), (0.4, 0.0)]))
        with self.assertRaises(DeskError):  # all near one spot
            fit_plane(tilted([(0.30, 0.20), (0.31, 0.20), (0.30, 0.21)]))
        bumpy = tilted(SPOTS)
        bumpy[1][2] += 0.02
        with self.assertRaises(DeskError):
            fit_plane(bumpy)

    def test_heights_are_relative_to_the_calibrated_spot(self):
        p = DeskPlane(0.02, -0.01, 0.123)  # the constant (e.g. jaw tip offset) cancels
        self.assertAlmostEqual(height_at(p, (0.3, 0.2), (0.35, 0.0), 0.004), 0.004 + 0.02 * 0.05 - 0.01 * -0.2)
        self.assertEqual(height_at(None, (0.3, 0.2), (0.35, 0.0), 0.004), 0.004)

    def test_box_spots_follow_the_desk(self):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        cfg = load_config(temp_config(Path(tmp.name)))
        box = Box(1, "available", "supply", None, box_frame((0.32, 0.16), 0.0, PAN), tool_in_box(), {}, {}, "", "")
        flat_pack = box_pose(box, "pack", cfg, PAN)[2, 3]
        self.assertEqual(flat_pack, 0.0)  # no desk plane: same height as the calibrated box
        DeskPlane(0.02, -0.01, 0.0).save(cfg.desk_file)
        pack, out = box_pose(box, "pack", cfg, PAN), box_pose(box, "output", cfg, PAN)
        self.assertAlmostEqual(pack[2, 3], 0.02 * (pack[0, 3] - 0.32) - 0.01 * (pack[1, 3] - 0.16))
        self.assertAlmostEqual(out[2, 3], -0.01 * (out[1, 3] - 0.16))  # mirrored spot: same x, other side
        self.assertEqual(box_pose(box, "supply", cfg, PAN)[2, 3], 0.0)  # the calibrated spot itself is unchanged


class ApiTests(unittest.TestCase):
    def test_capture_fit_save_remove(self):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        cfg = load_config(temp_config(Path(tmp.name)))
        arm = FakeArm(gain=0.9)
        hub = ArmHub(lambda: arm, "sim", lambda: MODEL, poll_s=0.01)
        hub.connect()
        self.addCleanup(hub.close)
        http = TestClient(create_server(OrderStore(":memory:"), Catalog(cfg.catalog, 10), shelf=Shelf(cfg), hub=hub))
        self.assertEqual(http.post("/api/desk/capture").status_code, 409)  # motors on
        hub.set_torque(False)
        touches = iter(np.array(tilted(SPOTS, noise=0.0003)))
        hub.read_tip = lambda *a, **k: next(touches)  # the jaw tip touching the desk at each spot
        for _ in SPOTS[:2]:
            r = http.post("/api/desk/capture").json()
        self.assertIn("at least 3", r["fit_error"])
        self.assertEqual(http.post("/api/desk/save").status_code, 422)
        for _ in SPOTS[2:]:
            r = http.post("/api/desk/capture").json()
        self.assertEqual(len(r["points"]), 4)
        self.assertAlmostEqual(r["fit"]["a"], 0.02, delta=0.005)
        r = http.post("/api/desk/save").json()
        self.assertTrue(cfg.desk_file.exists())
        self.assertEqual(r["points"], [])
        self.assertAlmostEqual(r["saved"]["b"], -0.01, delta=0.005)
        self.assertIsNone(http.delete("/api/desk").json()["saved"])
        self.assertFalse(cfg.desk_file.exists())
        self.assertEqual(http.delete("/api/desk/points").json()["points"], [])


if __name__ == "__main__":
    unittest.main()
