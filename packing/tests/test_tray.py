"""Parts tray: DXF layout, calibration fit, hand-guided calibration flow, pocket inventory, tray API."""

import math
import shutil
import tempfile
import unittest
from pathlib import Path

import ezdxf
import numpy as np
import yaml
from fastapi.testclient import TestClient
from scipy.spatial.transform import Rotation

from packing.cell.inventory import Inventory
from packing.config import load_config
from packing.hardware.arm import FakeArm
from packing.service.orders import OrderStore
from packing.service.vapi import Catalog
from packing.shop.server import create_server
from packing.shop.shelf import Shelf
from packing.tools.tray_calibrate import run_tray_calibration
from packing.tray.calibration import (CalibrationError, TrayCalibration, approach_misalignment_deg, calibrate,
                                      checks, tilt_deg)
from packing.tray.dxf import read_features
from packing.tray.layout import TrayType, build_layout

ROOT = Path(__file__).resolve().parents[2]
CELL = ROOT / "packing" / "cell.yaml"
PAN_XY = np.array([0.0388, 0.0])  # roughly where the SO-101 pan axis is (the real value comes from ArmModel)


def real_types():
    return load_config(CELL).tray.types


def temp_config(tmp: Path) -> Path:
    """Copy of cell.yaml whose databases/calibration live in `tmp` (never the real data files)."""
    raw = yaml.safe_load(CELL.read_text())
    raw["inventory_db"] = str(tmp / "inv.db")
    raw["poses_file"] = str(tmp / "poses.json")
    raw["tray"]["dxf"] = str(ROOT / "part.dxf")
    raw["tray"]["calibration_file"] = str(tmp / "tray_cal.json")
    path = tmp / "cell.yaml"
    path.write_text(yaml.safe_dump(raw))
    return path


def tilted_pose(tilt_deg_: float = 40, dist_m: float = 0.25):
    """A tray facing the arm (+x away from base), tilted back toward the ceiling, centered on the pan axis."""
    R = Rotation.from_euler("x", tilt_deg_, degrees=True).as_matrix()  # tray y leans up
    R = Rotation.from_euler("z", -90, degrees=True).as_matrix() @ R  # tray x -> robot -y (left->right seen from arm)
    center = np.array([89.44, 53.34, 0]) * 0.001
    t = np.array([PAN_XY[0] + dist_m, 0.0, 0.05]) - R @ center
    return R, t


class LayoutTests(unittest.TestCase):
    def test_real_tray(self):
        layout = build_layout(read_features(ROOT / "part.dxf"), real_types())
        counts = {}
        for p in layout.pockets:
            counts[p.sku] = counts.get(p.sku, 0) + 1
        self.assertEqual(counts, {"aa": 7, "9v": 5, "piezo": 8, "glue": 8, "breadboard": 5})
        self.assertEqual(layout.unmatched, [])
        self.assertEqual((layout.pocket("aa_1").x, layout.pocket("aa_1").y), (0.0, 0.0))
        self.assertAlmostEqual(layout.pocket("aa_7").y, 106.68)
        self.assertAlmostEqual(layout.pocket("piezo_3").x, 88.9)  # midpoint between the pins
        self.assertAlmostEqual(layout.pocket("9v_5").x, 50.8)
        self.assertAlmostEqual(layout.pocket("breadboard_5").y, 101.6)
        self.assertTrue(all(p.yaw_deg == 0.0 for p in layout.pockets))
        self.assertEqual(layout.outline["corner_r"], 12.0)
        bb = layout.pocket("breadboard_1")
        np.testing.assert_allclose(bb.grip_dir("y"), (0, 1), atol=1e-9)

    def test_polyline_rects_rotation_and_unmatched(self):
        with tempfile.TemporaryDirectory() as d:
            doc = ezdxf.new()
            m = doc.modelspace()
            m.add_lwpolyline([(-20, -20), (120, -20), (120, 80), (-20, 80)], close=True)  # plate
            m.add_lwpolyline([(0, 0), (27.5, 0), (27.5, 18.5), (0, 18.5)], close=True)  # 9V, as polyline
            m.add_lwpolyline([(50, 0), (68.5, 0), (68.5, 27.5), (50, 27.5)], close=True)  # 9V rotated 90°
            m.add_circle((90, 10), 15.5 / 2)  # AA
            m.add_circle((90, 40), 3)  # unknown hole
            m.add_line((0, 50), (10, 50)); m.add_arc((10, 52), 2, 270, 90)  # rounded slot: not a rect
            m.add_line((10, 54), (0, 54)); m.add_arc((0, 52), 2, 90, 270)
            path = Path(d) / "t.dxf"
            doc.saveas(path)
            types = [TrayType("9v", "9V", "rect", [27.5, 18.5]), TrayType("aa", "AA", "circle", 15.5)]
            layout = build_layout(read_features(path), types)
        self.assertEqual([p.id for p in layout.pockets], ["9v_1", "9v_2", "aa_1"])
        self.assertEqual(layout.pocket("9v_1").yaw_deg, 0.0)
        self.assertEqual(layout.pocket("9v_2").yaw_deg, 90.0)
        np.testing.assert_allclose(layout.pocket("9v_2").grip_dir("x"), (0, 1), atol=1e-9)
        self.assertEqual(len(layout.unmatched), 1)
        self.assertIn("d=6.00", layout.unmatched[0])

    def test_type_validation(self):
        with self.assertRaises(ValueError):
            TrayType("x", "X", "hexagon", [1])
        with self.assertRaises(ValueError):
            TrayType("x", "X", "rect", [1])
        with self.assertRaises(ValueError):
            TrayType("x", "X", "pin_pair", [1])  # no spacing


class CalibrationTests(unittest.TestCase):
    REFS = [("aa_1", (0, 0)), ("aa_7", (0, 106.68)), ("breadboard_1", (158.75, 0)), ("breadboard_5", (158.75, 101.6))]

    def measure(self, R, t, noise_mm=0.0, seed=0):
        rng = np.random.default_rng(seed)
        return [(pid, xy, R @ (np.array([*xy, 0]) * 0.001) + t + rng.normal(0, noise_mm * 0.001, 3))
                for pid, xy in self.REFS]

    def test_recovers_tilted_tray(self):
        R, t = tilted_pose(40)
        cal = calibrate(self.measure(R, t, noise_mm=0.5))
        self.assertLess(cal.rms_mm, 1.5)
        np.testing.assert_allclose(cal.rot, R, atol=0.02)
        np.testing.assert_allclose(cal.to_robot(158.75, 101.6), R @ [0.15875, 0.1016, 0] + t, atol=0.002)
        self.assertAlmostEqual(tilt_deg(cal), 40, delta=1.5)
        self.assertEqual(checks(cal, (89.44, 53.34), PAN_XY), [])

    def test_rejects_bad_points(self):
        R, t = tilted_pose()
        pts = self.measure(R, t)
        pts[2] = (pts[2][0], pts[2][1], pts[2][2] + [0.01, 0, 0])  # one point 10 mm off
        with self.assertRaises(CalibrationError) as ctx:
            calibrate(pts)
        self.assertIn("breadboard_1", str(ctx.exception))
        line = [(f"p{i}", (i * 10.0, 0.0), np.array([i * 0.01, 0, 0])) for i in range(4)]
        with self.assertRaises(CalibrationError):
            calibrate(line)

    def test_warns_when_not_facing_arm(self):
        R, t = tilted_pose()
        R2 = Rotation.from_euler("z", 40, degrees=True).as_matrix() @ R  # tray turned away
        cal = calibrate(self.measure(R2, t))
        self.assertTrue(any("doesn't face" in w for w in checks(cal, (89.44, 53.34), PAN_XY)))

    def test_grip_angle_grows_off_center(self):
        R, t = tilted_pose(40, dist_m=0.25)
        cal = calibrate(self.measure(R, t))
        center = approach_misalignment_deg(cal, 89.44, 53.34, PAN_XY)
        edge = approach_misalignment_deg(cal, 0, 53.34, PAN_XY)
        self.assertLess(center, 0.5)
        self.assertGreater(edge, 10)

    def test_save_load(self):
        R, t = tilted_pose()
        cal = calibrate(self.measure(R, t))
        with tempfile.TemporaryDirectory() as d:
            cal.save(Path(d) / "c.json")
            back = TrayCalibration.load(Path(d) / "c.json")
            self.assertIsNone(TrayCalibration.load(Path(d) / "missing.json"))
        np.testing.assert_allclose(back.rot, cal.rot)
        self.assertEqual(back.points[0].pocket, "aa_1")


class HandGuidedFlowTests(unittest.TestCase):
    def test_flow_with_redo_and_save(self):
        with tempfile.TemporaryDirectory() as d:
            cfg = load_config(temp_config(Path(d)))
            layout = cfg.tray.layout()
            R, t = tilted_pose()
            truth = {i: R @ (np.array([layout.pocket(pid).x, layout.pocket(pid).y, 0]) * 0.001) + t
                     for i, pid in enumerate(cfg.tray.reference_points)}
            arm = FakeArm()

            def put_tip_at(i):  # the "hand" moves the arm; pose encodes which point it is at
                arm.pos = arm.target = {**arm.pos, "shoulder_pan": float(i)}

            script = iter([
                ("", None),          # motors off
                ("", 0), ("b", None),  # point 1, then redo it
                ("", 0), ("", 1), ("", 2), ("", 3),
                ("y", None),         # save
            ])

            def input_fn(_prompt):
                answer, where = next(script)
                if where is not None:
                    put_tip_at(where)
                return answer

            out = []
            cal = run_tray_calibration(arm, lambda pose: truth[round(pose["shoulder_pan"])], cfg.tray, cfg.catalog,
                                       pan_xy=PAN_XY, input_fn=input_fn, out=out.append, sleep=lambda s: None)
            self.assertIsNotNone(cal)
            self.assertLess(cal.rms_mm, 0.01)
            self.assertTrue(cfg.tray.calibration_file.exists())
            self.assertTrue(arm.torque)  # motors back on at the end
            self.assertTrue(any("worst grip angle" in line for line in out))

    def test_quit_saves_nothing(self):
        with tempfile.TemporaryDirectory() as d:
            cfg = load_config(temp_config(Path(d)))
            arm = FakeArm()
            answers = iter(["", "q"])
            cal = run_tray_calibration(arm, lambda pose: np.zeros(3), cfg.tray, cfg.catalog,
                                       input_fn=lambda _p: next(answers), out=lambda s: None, sleep=lambda s: None)
            self.assertIsNone(cal)
            self.assertFalse(cfg.tray.calibration_file.exists())
            self.assertTrue(arm.torque)


class PocketInventoryTests(unittest.TestCase):
    def test_set_slot(self):
        inv = Inventory(":memory:")
        self.assertTrue(inv.set_slot("aa_1", "aa", True))
        self.assertFalse(inv.set_slot("aa_1", "aa", True))  # already full
        self.assertEqual(inv.full_slots(), {"aa_1"})
        part = inv.available_part("aa_1")
        inv.pick_part(part.id)  # robot takes it -> pocket is empty
        self.assertEqual(inv.full_slots(), set())
        self.assertTrue(inv.set_slot("aa_1", "aa", True))  # restocked
        self.assertTrue(inv.set_slot("aa_1", "aa", False))  # human took it out
        self.assertEqual(inv.full_slots(), set())

    def test_config_catalog_and_slots(self):
        cfg = load_config(CELL)
        self.assertEqual(cfg.catalog["glue"], "glue stick")
        self.assertEqual(len(cfg.stock_slots), 33)
        self.assertEqual(cfg.stock_slots["piezo_8"], "piezo")
        self.assertNotIn("aa_1.above", cfg.required_poses())  # pockets are computed, not taught


class TrayApiTests(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        tmp = Path(self._tmp.name)
        self.cfg = load_config(temp_config(tmp))
        self.shelf = Shelf(self.cfg)
        catalog = Catalog(self.cfg.catalog, max_items=1, available=self.shelf.available)
        self.http = TestClient(create_server(OrderStore(":memory:"), catalog, shelf=self.shelf))

    def tearDown(self):
        self._tmp.cleanup()

    def test_tray_state_and_marking(self):
        r = self.http.get("/api/tray").json()
        self.assertEqual(len(r["pockets"]), 33)
        self.assertIsNone(r["calibration"])
        self.assertNotIn("robot_m", r["pockets"][0])
        self.assertEqual(self.http.post("/api/tray/pockets/glue_2", json={"full": True}).json()["changed"], True)
        self.assertEqual(self.http.post("/api/tray/fill", json={"full": True, "sku": "aa"}).json()["changed"], 7)
        full = {p["id"] for p in self.http.get("/api/tray").json()["pockets"] if p["full"]}
        self.assertEqual(full, {"glue_2"} | {f"aa_{i}" for i in range(1, 8)})
        stock = {s["sku"]: s["on_shelf"] for s in self.http.get("/api/state").json()["stock"]}
        self.assertEqual((stock["aa"], stock["glue"], stock["9v"]), (7, 1, 0))
        self.assertEqual(self.http.post("/api/tray/fill", json={"full": False}).json()["changed"], 8)
        self.assertEqual(self.http.post("/api/tray/pockets/nope_1", json={"full": True}).status_code, 404)
        self.assertEqual(self.http.post("/api/tray/fill", json={"sku": "nope"}).status_code, 404)

    def test_calibrated_pockets_have_robot_positions(self):
        R, t = tilted_pose()
        pts = [(pid, xy, R @ (np.array([*xy, 0]) * 0.001) + t) for pid, xy in CalibrationTests.REFS]
        calibrate(pts).save(self.cfg.tray.calibration_file)
        r = self.http.get("/api/tray").json()
        self.assertAlmostEqual(r["calibration"]["tilt_deg"], 40, delta=0.5)
        p = next(p for p in r["pockets"] if p["id"] == "glue_3")
        np.testing.assert_allclose(p["robot_m"], R @ [0.1143, 0.03048, 0] + t, atol=1e-4)
        self.assertTrue(math.isclose(np.linalg.det(np.array(r["calibration"]["matrix"])[:3, :3]), 1, abs_tol=1e-6))


if __name__ == "__main__":
    unittest.main()
