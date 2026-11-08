"""Tray picking: motion programs (packing/motion/program.py) + pick planner (packing/tray/pick.py) + running a
pick on a FakeArm through the hub's runner + POST /api/arm/pick. Sim only."""

import tempfile
import time
import unittest
from pathlib import Path

import numpy as np
from fastapi.testclient import TestClient

from packing.config import GripperCfg, MotionCfg, load_config
from packing.hardware.arm import FakeArm
from packing.motion.controller import GraspError, Motion
from packing.motion.program import CheckHolding, Grip, JointMove, LineMove, OpenGripper, ProgramBuilder, run
from packing.poses import PoseStore
from packing.service.orders import OrderStore
from packing.service.vapi import Catalog
from packing.shop.arm_hub import ArmHub
from packing.shop.goto_runner import GotoRunner
from packing.shop.server import create_server
from packing.shop.shelf import Shelf
from packing.tests.test_arm_view import MODEL
from packing.tests.test_tray import CalibrationTests, temp_config, tilted_pose
from packing.tray.calibration import calibrate
from packing.tray.pick import plan_pocket_pick

HOME = dict.fromkeys(["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"], 0.0)


def real_like_cal(cfg):
    """A tray like the real cell (2026-11-07): face nearly vertical, facing the arm, ~33 cm out. Saved."""
    R, t = tilted_pose(86, 0.33)
    cal = calibrate([(p, xy, R @ (np.array([*xy, 0]) * 0.001) + t) for p, xy in CalibrationTests.REFS])
    cal.save(cfg.tray.calibration_file)
    return cal


class ProgramTests(unittest.TestCase):
    def _motion(self, arm):
        return Motion(arm, PoseStore(Path(tempfile.mkdtemp()) / "p.json"), MotionCfg(), GripperCfg(),
                      sleep=lambda s: None, log=lambda m: None)

    def test_builder_validates_and_run_plays(self):
        b = ProgramBuilder(MODEL, HOME)
        start_tip = b.tip
        b.line_by([0.0, 0.0, 0.03], 20.0, "up 30 mm")
        b.open_gripper()
        self.assertEqual([type(s) for s in b.steps], [LineMove, OpenGripper])
        self.assertAlmostEqual(b.steps[0].length_mm, 30.0, delta=0.5)
        arm = FakeArm(dict(HOME, gripper=0.0), gain=0.9)
        run(b.steps, self._motion(arm))
        np.testing.assert_allclose(MODEL.tip(arm.read()), start_tip + [0, 0, 0.03], atol=0.002)
        with self.assertRaises(Exception):  # 2 m away: unreachable at plan time, nothing runs
            ProgramBuilder(MODEL, HOME).line_by([2.0, 0, 0], 20.0, "too far")

    def test_on_fail_runs_only_for_grasp_errors(self):
        arm = FakeArm(dict(HOME, gripper=60.0), gain=0.9)  # nothing to grip -> GraspError
        ran = []
        steps = [Grip(3.0, on_fail=[OpenGripper("recover")])]
        with self.assertRaises(GraspError):
            run(steps, self._motion(arm), on_step=lambda s: ran.append(s.label))
        self.assertEqual(ran, ["grip", "recover"])
        arm.gripper_block = None
        with self.assertRaises(GraspError):  # CheckHolding: closed gripper = dropped
            run([CheckHolding(3.0)], self._motion(FakeArm(dict(HOME, gripper=0.0))))


class PickPlanTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls._tmp = tempfile.TemporaryDirectory()
        cls.cfg = load_config(temp_config(Path(cls._tmp.name)))
        cls.cal = real_like_cal(cls.cfg)

    @classmethod
    def tearDownClass(cls):
        cls._tmp.cleanup()

    def plan(self, pocket):
        return plan_pocket_pick(MODEL, self.cfg.tray, self.cfg.gripper, self.cal, pocket, HOME)

    def test_program_shape_and_geometry(self):
        t, n = self.cfg.tray, self.cal.normal
        for pocket in ("aa_1", "glue_8", "breadboard_3"):
            with self.subTest(pocket):
                plan = self.plan(pocket)
                kinds = [type(s) for s in plan.steps]
                self.assertEqual(kinds, [OpenGripper, JointMove, LineMove, Grip, LineMove, CheckHolding, LineMove,
                                         CheckHolding])
                pk = t.layout().pocket(pocket)
                c = self.cal.to_robot(pk.x, pk.y)
                _, approach, line_in, grip, pull, _, lift, _ = plan.steps
                # approach point: protrusion + approach_mm out along the normal
                tip = MODEL.tip(approach.pose)
                self.assertAlmostEqual(float((tip - c) @ n) * 1000, t.protrusion_mm + t.approach_mm, delta=1.0)
                # grasp: 0.15 in off the tray face, on the normal through the pocket
                g = MODEL.tip(line_in.path[-1])
                self.assertAlmostEqual(float((g - c) @ n) * 1000, t.grasp_standoff_mm, delta=0.5)
                self.assertLess(np.linalg.norm(g - (c + n * t.grasp_standoff_mm / 1000)) * 1000, 2.0)
                # jaws close along the type's grip direction (tool x), up to the 5-DOF tilt
                # (pitching down turns about the sideways jaw axis: sideways grips stay aligned; the breadboard's
                # up/down grip tilts with the pitch — reported as jaw_off_deg)
                jaw = MODEL.fk(line_in.path[-1])[:3, 0]
                want = self.cal.dir_to_robot(*pk.grip_dir(next(x.grip_axis for x in t.types if x.sku == pk.sku)))
                off = np.degrees(np.arccos(min(1.0, abs(float(jaw @ want)))))
                self.assertAlmostEqual(off, plan.jaw_off_deg, delta=0.5)
                self.assertLess(off, 16 if pk.sku != "breadboard" else plan.tilt_deg + 5)  # real cell: ≤15.5°
                # pull: straight out along the normal, at least extract_mm, orientation held
                p_end = MODEL.tip(pull.path[-1])
                self.assertGreaterEqual(plan.pull_mm, t.extract_mm)
                np.testing.assert_allclose(p_end, g + n * plan.pull_mm / 1000, atol=0.002)
                for pose in pull.path[:: max(1, len(pull.path) // 5)]:
                    off = p_end - MODEL.tip(pose)
                    self.assertLess(np.linalg.norm(off - n * float(off @ n)) * 1000, 2.0)  # stays on the line
                self.assertLess(plan.pull_twist_deg, 15.0)  # 5-DOF: tool yaw follows the pan (real cell: ≤12°)
                # lift: straight up
                np.testing.assert_allclose(MODEL.tip(lift.path[-1]), p_end + [0, 0, plan.lift_mm / 1000], atol=0.002)
                self.assertLessEqual(plan.pitch_deg, t.max_pitch_deg)

    def test_opens_to_part_width_plus_margin_not_fully(self):
        g = self.cfg.gripper
        for pocket, width in (("aa_1", 15.5), ("glue_2", 12.5), ("piezo_1", 12.0), ("breadboard_1", 2.5)):
            plan = self.plan(pocket)
            self.assertEqual(plan.grip_mm, width)
            self.assertEqual(plan.steps[0].value, plan.open_value)
            self.assertLess(plan.open_value, g.open / 2)  # far less than fully open
            # fingertip gap at that opening = width + margin (chord of the jaw swing)
            gap = 2 * g.jaw_length_mm * np.sin(np.radians(plan.open_value * g.deg_per_unit) / 2)
            self.assertAlmostEqual(gap, width + g.open_margin_mm, delta=0.3)
        self.assertEqual(GripperCfg(open=30).open_for(500), 30)  # capped at fully open

    def test_breadboard_grips_vertically_others_sideways(self):
        jaw = lambda plan: MODEL.fk(plan.steps[2].path[-1])[:3, 0]  # noqa: E731
        self.assertGreater(abs(jaw(self.plan("breadboard_2"))[2]), 0.6)  # tray y ~ up
        self.assertLess(abs(jaw(self.plan("aa_2"))[2]), 0.2)  # tray x ~ sideways

    def test_unreachable_tray_raises(self):
        from packing.motion.kinematics import PathError

        (Path(self._tmp.name) / "far").mkdir(exist_ok=True)
        far = load_config(temp_config(Path(self._tmp.name) / "far"))
        R, t = tilted_pose(86, 0.60)  # 60 cm out: beyond reach
        cal = calibrate([(p, xy, R @ (np.array([*xy, 0]) * 0.001) + t) for p, xy in CalibrationTests.REFS])
        with self.assertRaises(PathError):
            plan_pocket_pick(MODEL, far.tray, far.gripper, cal, "aa_4", HOME)


class PickRunTests(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.cfg = load_config(temp_config(Path(self._tmp.name)))
        self.cal = real_like_cal(self.cfg)
        self.arm = FakeArm(gain=0.9)
        self.hub = ArmHub(lambda: self.arm, "sim", lambda: MODEL, poll_s=0.01)
        self.hub.connect()
        self.events = []
        self.runner = GotoRunner(self.hub, self.cfg, lambda k, t, **d: self.events.append(t), sleep=lambda s: None)

    def tearDown(self):
        self.hub.mode = "idle"
        self.hub.close()
        self._tmp.cleanup()

    def test_pick_succeeds_and_holds_part_high(self):
        plan = plan_pocket_pick(MODEL, self.cfg.tray, self.cfg.gripper, self.cal, "glue_4", self.arm.read())
        self.arm.gripper_block = plan.open_value - 4  # a part between the jaws, a bit narrower than the opening
        self.runner.start(plan)
        self.runner.join(30)
        self.assertEqual(self.runner.last["status"], "done", self.runner.last)
        np.testing.assert_allclose(MODEL.tip(self.arm.read()), MODEL.tip(plan.steps[-2].path[-1]), atol=0.003)
        self.assertAlmostEqual(self.arm.read()["gripper"], plan.open_value - 4, delta=0.5)
        self.assertAlmostEqual(self.runner.last["grip_width"], plan.open_value - 4, delta=0.5)
        self.assertTrue(any("gripped at width" in e for e in self.events))
        self.assertEqual(self.hub.mode, "idle")

    def test_missed_grip_backs_out_and_fails(self):
        plan = plan_pocket_pick(MODEL, self.cfg.tray, self.cfg.gripper, self.cal, "aa_3", self.arm.read())
        self.runner.start(plan)  # no gripper_block: the jaws close on nothing
        self.runner.join(30)
        self.assertEqual(self.runner.last["status"], "failed")
        self.assertIn("nothing grasped", self.runner.last["error"])
        approach = MODEL.tip(plan.steps[1].pose)
        np.testing.assert_allclose(MODEL.tip(self.arm.read()), approach, atol=0.003)  # backed out
        self.assertAlmostEqual(self.arm.read()["gripper"], plan.open_value, delta=1.0)  # opened again, part-wide

    def test_api_dry_run_then_move(self):
        http = TestClient(create_server(OrderStore(":memory:"), Catalog(self.cfg.catalog, 1), shelf=Shelf(self.cfg),
                                        hub=self.hub))
        r = http.post("/api/arm/pick", json={"pocket": "piezo_5"})
        self.assertEqual(r.status_code, 200, r.text)
        r = r.json()
        self.assertFalse(r["started"])
        self.assertEqual(r["plan"]["sku"], "piezo")
        self.assertEqual([s["step"] for s in r["plan"]["steps"]][:4], ["OpenGripper", "JointMove", "LineMove", "Grip"])
        self.assertEqual(http.post("/api/arm/pick", json={"pocket": "corner_bl"}).status_code, 404)
        self.arm.gripper_block = r["plan"]["open_value"] - 4
        self.assertTrue(http.post("/api/arm/pick", json={"pocket": "piezo_5", "confirm_move": True}).json()["started"])
        deadline = time.monotonic() + 60  # runner shared with go-to: its state endpoint reports the pick
        while (st := http.get("/api/arm/goto").json())["running"] and time.monotonic() < deadline:
            time.sleep(0.05)
        self.assertEqual(st["last"]["status"], "done", st)


if __name__ == "__main__":
    unittest.main()
