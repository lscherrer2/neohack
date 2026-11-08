"""Joint zero offsets fitted from desk touches (packing/motion/joint_offsets.py, plan
.claude/plans/joint_offset_fit.md): the model with offsets, recovering a known zero error, refusals, and the
dashboard API (fit → apply → desk points / plane / boxes re-derived → remove). Sim only."""

import tempfile
import unittest
from pathlib import Path

import numpy as np
from fastapi.testclient import TestClient

from packing.boxes.desk import JAW_TIP_M, DeskPlane, touch_point
from packing.boxes.geometry import box_frame
from packing.boxes.move import reach_seeds
from packing.boxes.store import BoxStore
from packing.config import load_config
from packing.hardware.arm import FakeArm
from packing.motion.joint_offsets import OffsetFitError, fit_offsets, jaw_tips, load_offsets, save_offsets
from packing.motion.kinematics import ArmModel, arm_model, load_ranges
from packing.service.orders import OrderStore
from packing.service.vapi import Catalog
from packing.shop.arm_hub import ArmHub
from packing.shop.server import create_server
from packing.shop.shelf import Shelf
from packing.tests.test_arm_view import MODEL
from packing.tests.test_boxes import tool_in_box
from packing.tests.test_tray import temp_config

RANGES = load_ranges("my_awesome_follower_arm")
TRUE_OFFSETS = {"shoulder_lift": 1.5, "elbow_flex": -1.0}  # the "real" arm's zero error
TRUE = ArmModel(RANGES, offsets_deg=TRUE_OFFSETS)
DESK_Z = 0.003
DOWN = np.diag([1.0, -1.0, -1.0])


def touch_pose(model: ArmModel, x: float, y: float, z: float = DESK_Z) -> dict:
    """Joint pose whose jaw tip (per `model`) touches (x, y, z) — gripper pitched however reaches."""
    goal = np.array([x, y, z])
    p = goal + [0, 0, JAW_TIP_M]
    best = None
    for s in reach_seeds(model, p):
        q, err = model.ik(s, model.target_at(DOWN, p), orientation_weight=0.0)
        if all(abs(v) <= 100 for v in q.values()) and (best is None or err < best[1]):
            best = (q, err)
    q = best[0]
    for _ in range(6):  # aim the jaw tip, not the tool point
        p = p + goal - touch_point(model.fk(q))
        q, _ = model.ik(q, model.target_at(DOWN, p), orientation_weight=0.0)
    assert np.linalg.norm(touch_point(model.fk(q)) - goal) < 2e-4
    return q


def desk_touches(model=TRUE, radii=(0.17, 0.24, 0.31, 0.37), azimuths=(-55, 0, 55)) -> list[dict]:
    out = []
    for r in radii:
        for az in azimuths:
            a = np.radians(az)
            x, y = MODEL.pan_xy + r * np.array([np.cos(a), np.sin(a)])
            out.append(touch_pose(model, x, y))
    return out


TOUCHES = desk_touches()  # flat desk as the REAL (offset) arm touches it


class ModelTests(unittest.TestCase):
    def test_offsets_shift_fk_and_ik_consistently(self):
        q = {"shoulder_pan": 0.0, "shoulder_lift": 20.0, "elbow_flex": 30.0, "wrist_flex": 40.0, "wrist_roll": 0.0}
        self.assertGreater(np.linalg.norm(TRUE.tip(q) - MODEL.tip(q)), 0.002)
        np.testing.assert_allclose(TRUE.to_deg(q) - MODEL.to_deg(q), [0, 1.5, -1.0, 0, 0], atol=1e-9)
        back = TRUE.to_units(TRUE.to_deg(q))
        self.assertAlmostEqual(back["shoulder_lift"], 20.0)

    def test_arm_model_reads_the_saved_offsets(self):
        with tempfile.TemporaryDirectory() as d:
            f = Path(d) / "offsets.json"
            self.assertEqual(arm_model("my_awesome_follower_arm", f).offsets_deg, {})
            save_offsets(f, {"shoulder_lift": 0.7})
            self.assertEqual(load_offsets(f), {"shoulder_lift": 0.7})
            self.assertEqual(arm_model("my_awesome_follower_arm", f).offsets_deg, {"shoulder_lift": 0.7})


class FitTests(unittest.TestCase):
    def test_uncorrected_touches_rise_with_reach(self):
        z = jaw_tips(MODEL, TOUCHES)[:, 2]
        self.assertGreater(z[-3:].mean() - z[:3].mean(), 0.003)  # far touches read mm higher than near ones

    def test_many_touches_recover_both_joints(self):
        touches = desk_touches(radii=(0.16, 0.22, 0.28, 0.34, 0.39), azimuths=(-60, -30, 0, 30, 60))
        rep = fit_offsets(MODEL, touches)
        self.assertTrue(rep["ok"], rep["problems"])
        self.assertEqual(rep["joints"], ["shoulder_lift", "elbow_flex"])
        self.assertAlmostEqual(rep["delta_deg"]["shoulder_lift"], 1.5, delta=0.15)
        self.assertAlmostEqual(rep["delta_deg"]["elbow_flex"], -1.0, delta=0.2)
        self.assertLess(rep["rms_after_mm"], 0.1)
        self.assertEqual([p["r_mm"] for p in rep["points"]], sorted(p["r_mm"] for p in rep["points"]))
        z = jaw_tips(ArmModel(RANGES, offsets_deg=rep["total_deg"]), touches)[:, 2]
        self.assertLess(z.max() - z.min(), 0.0005)  # flat again

    def test_fewer_touches_fall_back_to_the_lift_alone(self):
        rep = fit_offsets(MODEL, TOUCHES)  # 12 touches: lift and elbow can't be told apart to ±0.5°
        self.assertTrue(rep["ok"], rep["problems"])
        self.assertEqual(rep["joints"], ["shoulder_lift"])
        self.assertIn("alone", rep["note"])
        self.assertAlmostEqual(rep["delta_deg"]["shoulder_lift"], 1.2, delta=0.3)
        self.assertLess(rep["rms_after_mm"], rep["rms_before_mm"] / 3)  # still mostly flattens it

    def test_fit_is_relative_to_the_current_offsets(self):
        part = ArmModel(RANGES, offsets_deg={"shoulder_lift": 1.0})
        rep = fit_offsets(part, TOUCHES)
        self.assertGreater(rep["total_deg"]["shoulder_lift"], 1.0)  # on top of the current 1°, toward the true 1.5°
        self.assertAlmostEqual(rep["total_deg"]["shoulder_lift"], 1.5, delta=0.3)
        self.assertAlmostEqual(rep["current_deg"]["shoulder_lift"], 1.0)

    def test_already_flat_is_not_ok(self):
        rep = fit_offsets(TRUE, TOUCHES)  # the right model: nothing to fix
        self.assertFalse(rep["ok"])

    def test_refuses_few_or_badly_spread_touches(self):
        with self.assertRaises(OffsetFitError):
            fit_offsets(MODEL, TOUCHES[:5])
        with self.assertRaisesRegex(OffsetFitError, "far"):
            fit_offsets(MODEL, desk_touches(radii=(0.30, 0.33), azimuths=(-55, 0, 55)))
        with self.assertRaisesRegex(OffsetFitError, "around"):
            fit_offsets(MODEL, desk_touches(azimuths=(-5, 5)))


class ApiTests(unittest.TestCase):
    def setUp(self):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        self.cfg = cfg = load_config(temp_config(Path(tmp.name)))
        cfg.joint_offsets_file = Path(tmp.name) / "offsets.json"
        arm = FakeArm(gain=0.9)
        self.hub = ArmHub(lambda: arm, "sim", cfg.arm_model, poll_s=0.01)
        self.hub.connect()
        self.addCleanup(self.hub.close)
        self.store = OrderStore(":memory:")
        self.http = TestClient(create_server(self.store, Catalog(cfg.catalog, 10), shelf=Shelf(cfg), hub=self.hub))
        self.hub.set_torque(False)

    def capture(self, poses):
        it = iter(poses)
        self.hub.read_pose = lambda *a, **k: {**next(it), "gripper": 0.0}
        for _ in poses:
            r = self.http.post("/api/desk/capture")
            self.assertEqual(r.status_code, 200, r.text)
        return r.json()

    def test_fit_apply_rederive_remove(self):
        http, cfg = self.http, self.cfg
        self.assertEqual(http.post("/api/joint-offsets/fit").status_code, 422)  # no touches yet
        # a box calibrated with the uncorrected model (its capture joints are kept)
        T_box0 = box_frame((0.32, 0.16), 0.0, MODEL.pan_xy)
        T_grip = T_box0 @ tool_in_box()  # the level tab grip, solved with the uncorrected model
        q_box = min((MODEL.ik(s, T_grip, orientation_weight=1.0) for s in reach_seeds(MODEL, T_grip[:3, 3])),
                    key=lambda t: t[1])[0]
        boxes = BoxStore(self.store)
        boxes.add(T_box0, np.linalg.inv(T_box0) @ MODEL.fk(q_box), q_box, {})
        self.capture(TOUCHES)
        r = http.post("/api/desk/save").json()  # saved with the joint poses
        self.assertTrue(r["saved"]["has_poses"])
        tilt0 = r["saved"]["tilt_deg"]
        self.capture(TOUCHES[:4])
        http.delete("/api/desk/points")  # the fit falls back to the saved calibration's touches
        r = http.post("/api/joint-offsets/fit").json()
        self.assertEqual(r["offset_fit"]["source"], "the saved desk calibration")
        self.assertTrue(r["offset_fit"]["ok"])
        self.assertFalse(cfg.joint_offsets_file.exists())  # dry run
        r = http.post("/api/joint-offsets/apply")
        self.assertEqual(r.status_code, 200, r.text)
        r = r.json()
        self.assertFalse(any("box" in n for n in r["notes"]), r["notes"])
        lift = load_offsets(cfg.joint_offsets_file)["shoulder_lift"]
        self.assertAlmostEqual(lift, r["offset_fit"]["total_deg"]["shoulder_lift"])
        self.assertGreater(lift, 0.9)
        self.assertEqual(self.hub.model.offsets_deg["shoulder_lift"], lift)  # live model reloaded
        self.assertLess(r["saved"]["rms_mm"], 0.35)  # desk plane re-derived: (nearly) flat now
        self.assertLessEqual(r["saved"]["tilt_deg"], tilt0)
        b = boxes.get(1)
        self.assertGreater(np.linalg.norm(b.supply[:3, 3] - T_box0[:3, 3]), 0.0005)  # box re-derived from joints
        np.testing.assert_allclose(b.supply @ b.tool_in_box, self.hub.model.fk(q_box), atol=1e-9)
        self.assertEqual(http.post("/api/joint-offsets/apply").status_code, 422)  # already flat: nothing to apply
        r = http.delete("/api/joint-offsets").json()
        self.assertFalse(cfg.joint_offsets_file.exists())
        self.assertEqual(self.hub.model.offsets_deg, {})
        self.assertEqual(r["joint_offsets"]["offsets_deg"], {})
        self.assertGreater(r["saved"]["rms_mm"], 0.5)  # back to the uncorrected (rising) touches

    def test_old_desk_file_without_poses(self):
        DeskPlane(0.0, 0.0, 0.0, [[0.3, 0.1, 0.0]] * 3).save(self.cfg.desk_file)
        r = self.http.post("/api/joint-offsets/fit")
        self.assertEqual(r.status_code, 422)
        self.assertIn("re-touch", r.json()["detail"])


if __name__ == "__main__":
    unittest.main()
