"""Desk plane calibration (packing/boxes/desk.py): fit, refusals, box spot heights following a tilted desk, and the
API on a FakeArm. Sim only."""

import tempfile
import unittest
from pathlib import Path

import numpy as np
from fastapi.testclient import TestClient

from packing.boxes.desk import (JAW_TIP_M, DeskError, DeskPlane, box_below_desk, check_desk_clear, corrections_mm,
                                fit_plane, height_at, touch_point)
from packing.boxes.geometry import CaptureError, box_frame, box_from_grasp, obstacles, tray_blocks
from packing.boxes.move import box_pose, plan_box_move
from packing.boxes.store import Box
from packing.config import load_config
from packing.hardware.arm import FakeArm
from packing.motion.program import ProgramBuilder
from packing.service.orders import OrderStore
from packing.service.vapi import Catalog
from packing.shop.arm_hub import ArmHub
from packing.shop.server import create_server
from packing.shop.shelf import Shelf
from packing.tests.test_arm_view import MODEL
from packing.tests.test_boxes import tool_in_box
from packing.tests.test_tray import temp_config

PAN = MODEL.pan_xy
HOME = dict.fromkeys(["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"], 0.0)


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


class FloorTests(unittest.TestCase):
    """The plane used absolutely, as the floor: capture check, tray stand, desk collisions. No plane = as before."""

    def setUp(self):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        self.cfg = load_config(temp_config(Path(tmp.name)))

    def test_touch_point_is_the_jaw_tip(self):
        T = np.diag([1.0, -1.0, -1.0, 1.0])
        T[:3, 3] = [0.3, 0.0, 0.05]
        np.testing.assert_allclose(touch_point(T), [0.3, 0.0, 0.05 - JAW_TIP_M])

    def test_capture_compares_the_box_bottom_with_the_desk(self):
        T_tool = box_frame((0.32, 0.16), 0.03, PAN) @ tool_in_box()  # box standing on a desk 30 mm up
        with self.assertRaises(CaptureError):
            box_from_grasp(T_tool, PAN)  # vs z = 0: 30 mm off
        T_box, rep = box_from_grasp(T_tool, PAN, desk=DeskPlane(0.0, 0.0, 0.028))
        self.assertAlmostEqual(T_box[2, 3], 0.03, places=4)
        self.assertAlmostEqual(rep["above_desk_mm"], 2.0, delta=0.1)

    def test_tray_stand_goes_down_to_the_desk(self):
        from packing.tests.test_pack import left_tray

        left_tray(self.cfg)
        stand = lambda blocks: [b for b in blocks if "stand" in b.name]  # noqa: E731
        flat = stand(tray_blocks(self.cfg.tray))
        low = stand(tray_blocks(self.cfg.tray, DeskPlane(0.0, 0.0, -0.05)))
        self.assertTrue(low)
        lo = lambda b: float(b.T[2, 3] - b.half[2])  # noqa: E731
        self.assertAlmostEqual(lo(low[0]), -0.05, places=4)
        if flat:
            self.assertAlmostEqual(lo(flat[0]), 0.0, places=4)
        DeskPlane(0.0, 0.0, -0.05).save(self.cfg.desk_file)  # obstacles() reads the saved plane
        self.assertAlmostEqual(min(lo(b) for b in stand(obstacles(self.cfg, {}))), -0.05, places=4)

    def test_moves_into_the_desk_are_refused(self):
        b = ProgramBuilder(MODEL, HOME)
        start = dict(b.pose)
        tip = MODEL.tip(start)
        desk = DeskPlane(0.0, 0.0, float(tip[2]) - 0.02)  # desk 2 cm under the tool point
        check_desk_clear(MODEL, start, [], desk, "nothing")
        b.line_by(np.array([0.0, 0.0, -0.04]), 30.0, "down 4 cm")
        check_desk_clear(MODEL, start, b.steps, None, "no desk calibration")  # no-op
        with self.assertRaises(Exception) as e:
            check_desk_clear(MODEL, start, b.steps, desk, "test")
        self.assertIn("desk", str(e.exception))
        check_desk_clear(MODEL, start, b.steps, DeskPlane(0.0, 0.0, -0.02), "desk under the base")

    def test_box_corners_vs_the_desk(self):
        T = box_frame((0.32, 0.16), 0.03, PAN)
        self.assertEqual(box_below_desk(T, None), 0.0)
        self.assertLess(box_below_desk(T, DeskPlane(0.0, 0.0, 0.0)), 0.0)
        self.assertGreater(box_below_desk(T, DeskPlane(0.0, 0.0, 0.03)), 0.0)

    def test_box_move_plans_with_a_desk_and_corrections(self):
        box = Box(1, "available", "supply", None, box_frame((0.316, 0.16), 0.0, PAN), tool_in_box(), {}, {}, "", "")
        DeskPlane(0.01, -0.005, 0.0).save(self.cfg.desk_file)
        plan = plan_box_move(MODEL, box, "supply", "pack", HOME, self.cfg)
        self.assertTrue(plan.steps)
        c = corrections_mm(self.cfg.desk_plane(), (0.316, 0.16), {"pack": (0.355, 0.0)})
        self.assertAlmostEqual(c["pack"], (0.01 * 0.039 - 0.005 * -0.16) * 1000, places=1)
        self.assertEqual(corrections_mm(None, (0, 0), {"pack": (1, 1)}), {})

    def test_whole_order_checked_against_the_desk(self):
        from packing.boxes.pack import plan_pack_order
        from packing.tests.test_pack import SUPPLY, left_tray, order

        cal = left_tray(self.cfg)  # the sim tray's lowest pockets sit at z ≈ −3 mm
        box = Box(1, "available", "supply", None, box_frame(SUPPLY, 0.0, PAN), tool_in_box(), {}, {}, "", "")
        o = order(1, ["aa_1", "glue_3"], ["aa", "glue"])
        DeskPlane(0.0, 0.0, -0.012).save(self.cfg.desk_file)  # desk under the tray: plans
        plan_pack_order(MODEL, self.cfg, cal, o, box, HOME, obstacles(self.cfg, {}))
        DeskPlane(0.0, 0.0, 0.0).save(self.cfg.desk_file)  # desk above the lowest pockets: the pick would dig in
        with self.assertRaises(Exception) as e:
            plan_pack_order(MODEL, self.cfg, cal, o, box, HOME, obstacles(self.cfg, {}))
        self.assertIn("desk", str(e.exception))


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

        def pointing_down(*a, **k):  # the jaw tip touching the desk at each spot, gripper pointing straight down
            T = np.diag([1.0, -1.0, -1.0, 1.0])
            T[:3, 3] = next(touches) + [0, 0, JAW_TIP_M]  # tool point = JAW_TIP_M above the tip
            return dict(HOME), T

        hub.read_tool = pointing_down
        for _ in SPOTS[:2]:
            r = http.post("/api/desk/capture").json()
        self.assertIn("at least 3", r["fit_error"])
        self.assertEqual(http.post("/api/desk/save").status_code, 422)
        for _ in SPOTS[2:]:
            r = http.post("/api/desk/capture").json()
        self.assertEqual(len(r["points"]), 4)
        self.assertAlmostEqual(r["fit"]["a"], 0.02, delta=0.005)
        self.assertAlmostEqual(r["fit"]["c"], 0.006, delta=0.001)  # the jaw tips, i.e. the desk surface itself
        r = http.post("/api/desk/save").json()
        self.assertTrue(cfg.desk_file.exists())
        self.assertEqual(r["points"], [])
        self.assertAlmostEqual(r["saved"]["b"], -0.01, delta=0.005)
        self.assertIsNone(http.delete("/api/desk").json()["saved"])
        self.assertFalse(cfg.desk_file.exists())
        self.assertEqual(http.delete("/api/desk/points").json()["points"], [])


if __name__ == "__main__":
    unittest.main()
