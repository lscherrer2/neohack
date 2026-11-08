"""Reach compensation (packing/motion/reach_comp.py): aim higher the further out, applied at run time only.
Sim only."""

import dataclasses
import tempfile
import unittest
from pathlib import Path

import numpy as np
from fastapi.testclient import TestClient

from packing.config import load_config
from packing.hardware.arm import FakeArm
from packing.motion import reach_comp as rc_mod
from packing.motion.program import Grip, JointMove, ProgramBuilder
from packing.motion.reach_comp import ReachComp, lift_m, max_reach_m, radius, raise_pose, raise_steps
from packing.service.orders import OrderStore
from packing.service.vapi import Catalog
from packing.shop.arm_hub import ArmHub
from packing.shop.goto_runner import GotoRunner
from packing.shop.server import create_server
from packing.shop.shelf import Shelf
from packing.tests.test_arm_view import MODEL
from packing.tests.test_pick import real_like_cal
from packing.tests.test_tray import temp_config
from packing.tray.pick import plan_pocket_pick

ON = ReachComp(enabled=True, lift_mm=12.7)  # ½ in at 70 % of max reach, power 2 (default height is 6 mm)
Q = {"shoulder_pan": 0.0, "shoulder_lift": 20.0, "elbow_flex": 30.0, "wrist_flex": 40.0, "wrist_roll": 0.0}


class CurveTests(unittest.TestCase):
    def test_half_inch_at_seventy_percent(self):
        m = max_reach_m(MODEL)
        self.assertGreater(m, 0.35)
        self.assertAlmostEqual(lift_m(MODEL, ON, 0.7 * m), 0.0127, places=6)
        self.assertLess(lift_m(MODEL, ON, 0.2 * m), 0.0015)  # ~nothing near the swivel
        self.assertGreater(lift_m(MODEL, ON, 0.9 * m), 0.0127)  # more further out
        self.assertEqual(lift_m(MODEL, ReachComp(), 0.7 * m), 0.0)  # off by default
        self.assertAlmostEqual(lift_m(MODEL, ReachComp(enabled=True), 0.7 * m), 0.006, places=6)  # default 6 mm

    def test_raised_pose_is_higher_by_the_curve_same_orientation(self):
        T0 = MODEL.fk(Q)
        q = raise_pose(MODEL, ON, Q)
        T1 = MODEL.fk(q)
        d = T1[:3, 3] - T0[:3, 3]
        self.assertAlmostEqual(d[2], lift_m(MODEL, ON, radius(MODEL, T0[:3, 3])), delta=0.0007)
        self.assertLess(np.linalg.norm(d[:2]), 0.001)
        self.assertGreater(float(T0[:3, 2] @ T1[:3, 2]), 0.999)  # tool still points the same way

    def test_steps_raised_and_off_is_a_no_op(self):
        b = ProgramBuilder(MODEL, Q)
        b.line_by(np.array([0.03, 0.0, 0.0]), 30.0, "out 3 cm")
        steps = [JointMove(dict(Q), None, "go"), *b.steps, Grip(5.0, "grip", on_fail=[JointMove(dict(Q))])]
        self.assertIs(raise_steps(MODEL, steps, ReachComp()), steps)
        out = raise_steps(MODEL, steps, ON)
        self.assertEqual(len(out), len(steps))
        self.assertGreater(MODEL.tip(out[0].pose)[2], MODEL.tip(Q)[2] + 0.004)
        for q0, q1 in zip(steps[1].path, out[1].path):
            self.assertGreater(MODEL.tip(q1)[2] - MODEL.tip(q0)[2], 0.004)
        self.assertGreater(MODEL.tip(out[2].on_fail[0].pose)[2], MODEL.tip(Q)[2] + 0.004)
        self.assertEqual(steps[0].pose, Q)  # the plan itself is untouched

    def test_sleep_and_near_stretch_poses_are_raised(self):
        # Regression (2026-11-07): the sleep pose (gripper straight up) and stretched-out poses are near a singularity;
        # the old orientation-weighted placo IK stalled 1-3 mm short there and refused every pick test as "out of reach".
        from packing.poses import DEFAULTS

        rc = ReachComp(enabled=True)  # default 6 mm
        for q in (DEFAULTS["sleep"], {"shoulder_pan": 0.0, "shoulder_lift": -54.0, "elbow_flex": -74.0,
                                      "wrist_flex": 14.0, "wrist_roll": 0.0}):
            T0 = MODEL.fk(q)
            T1 = MODEL.fk(raise_pose(MODEL, rc, q))
            dz = lift_m(MODEL, rc, radius(MODEL, T0[:3, 3]))
            self.assertLess(np.linalg.norm(T1[:3, 3] - T0[:3, 3] - [0, 0, dz]), rc_mod.MAX_IK_ERR_M)
            self.assertLess(rc_mod._tilt_change_deg(T0[:3, :3], T1[:3, :3]), rc_mod.MAX_TILT_CHANGE_DEG)

    def test_out_of_reach_is_refused(self):
        with self.assertRaisesRegex(ValueError, "reach compensation"):
            raise_pose(MODEL, dataclasses.replace(ON, lift_mm=400.0), Q)  # 30+ cm up from a far pose


class RunAndApiTests(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmp.cleanup)
        self.cfg = load_config(temp_config(Path(self._tmp.name)))
        self.cfg.reach_comp_file = Path(self._tmp.name) / "reach_comp.json"
        self.cal = real_like_cal(self.cfg)
        self.arm = FakeArm(gain=0.9)
        self.hub = ArmHub(lambda: self.arm, "sim", lambda: MODEL, poll_s=0.01)
        self.hub.connect()
        self.addCleanup(self.hub.close)

    def _pick_end_z(self) -> float:
        runner = GotoRunner(self.hub, self.cfg, lambda *a, **k: None, sleep=lambda s: None)
        plan = plan_pocket_pick(MODEL, self.cfg.tray, self.cfg.gripper, self.cal, "glue_4", self.arm.read())
        self.arm.gripper_block = plan.open_value - 4
        runner.start(plan)
        runner.join(30)
        self.assertEqual(runner.last["status"], "done", runner.last)
        return float(MODEL.tip(self.arm.read())[2]), plan

    def test_toggle_via_api_raises_the_run(self):
        http = TestClient(create_server(OrderStore(":memory:"), Catalog(self.cfg.catalog, 10), shelf=Shelf(self.cfg),
                                        hub=self.hub))
        r = http.get("/api/reach-comp").json()
        self.assertFalse(r["enabled"])
        self.assertEqual([t["reach"] for t in r["table"]], [0.3, 0.5, 0.7, 0.9])
        z_off, plan = self._pick_end_z()
        r = http.put("/api/reach-comp", json={"enabled": True, "lift_mm": 10}).json()
        self.assertTrue(r["enabled"])
        self.assertEqual(r["lift_mm"], 10)
        self.assertTrue(self.cfg.reach_comp_file.exists())
        z_on, _ = self._pick_end_z()
        end = MODEL.tip(plan.steps[-2].path[-1])
        expect = lift_m(MODEL, rc_mod.load(self.cfg.reach_comp_file, ReachComp()), radius(MODEL, end))
        self.assertAlmostEqual(z_on - z_off, expect, delta=0.003)
        self.assertEqual(http.put("/api/reach-comp", json={"lift_mm": 99}).status_code, 422)
        self.assertFalse(http.put("/api/reach-comp", json={"enabled": False}).json()["enabled"])


if __name__ == "__main__":
    unittest.main()
