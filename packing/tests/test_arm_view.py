"""Live arm view + dashboard calibration: TelemetryArm, ArmHub, CalibrationSession, /api/arm + /api/calibration.

Sim only (FakeArm). The real ArmModel (placo) is used for FK where possible.
"""

import tempfile
import time
import unittest
from pathlib import Path

import numpy as np
from fastapi.testclient import TestClient

from packing.config import load_config
from packing.hardware.arm import FakeArm
from packing.hardware.telemetry import TelemetryArm
from packing.motion.kinematics import ArmModel, load_ranges
from packing.service.orders import OrderStore
from packing.service.vapi import Catalog
from packing.shop.arm_hub import ArmBusy, ArmHub
from packing.shop.server import create_server
from packing.shop.shelf import Shelf
from packing.tests import test_tray
from packing.tests.test_tray import temp_config, tilted_pose
from packing.tray.session import CalibrationSession, corner_label

MODEL = ArmModel(load_ranges("my_awesome_follower_arm"))


class FakeTipModel:
    """Stands in for ArmModel in calibration tests: the 'hand' sets shoulder_pan = index of a known point."""

    pan_xy = np.array([0.0388, 0.0])

    def __init__(self, points):
        self.points = points

    def tip(self, pose):
        return self.points[round(pose["shoulder_pan"])]

    def link_frames(self, pose):
        T = np.eye(4)
        T[:3, 3] = self.tip(pose) if round(pose["shoulder_pan"]) in self.points else 0
        return {"base_link": np.eye(4), "gripper_frame_link": T}


def wait_for(cond, timeout=2.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if cond():
            return True
        time.sleep(0.01)
    return False


class TelemetryArmTests(unittest.TestCase):
    def test_throttles_and_never_raises(self):
        now = [0.0]
        sent = []
        arm = TelemetryArm(FakeArm(), lambda j, t: sent.append((j, t)), min_interval_s=0.1, clock=lambda: now[0])
        arm.read(); arm.read()
        now[0] = 0.15
        arm.read()
        self.assertEqual(len(sent), 2)
        arm.set_torque(False)
        now[0] = 1.0
        arm.read()
        self.assertFalse(sent[-1][1])

        def boom(j, t):
            raise RuntimeError("server down")

        TelemetryArm(FakeArm(), boom).read()  # must not raise


class ArmHubTests(unittest.TestCase):
    def make(self, **kw):
        hub = ArmHub(lambda: FakeArm(), "sim", lambda: MODEL, poll_s=0.01, **kw)
        self.addCleanup(lambda: hub.mode == "idle" and hub.close())
        return hub

    def test_poll_publishes_fk_frames(self):
        hub = self.make()
        self.assertFalse(hub.state()["connected"])
        hub.connect()
        self.assertTrue(wait_for(lambda: hub.state()["frames"] is not None))
        st = hub.state()
        self.assertEqual((st["connected"], st["source"], st["torque"], st["mode"]), (True, "sim", True, "idle"))
        self.assertEqual(list(st["frames"]), ArmModel.LINKS)
        np.testing.assert_allclose(st["tip"], MODEL.tip(dict.fromkeys(st["joints"], 0.0)), atol=1e-4)
        self.assertEqual(len(st["frames"]["wrist_link"]), 4)

    def test_borrow_marks_packing_and_keeps_arm_open(self):
        hub = self.make()
        with hub.borrow() as arm:
            self.assertEqual(hub.state()["mode"], "packing")
            arm.write({**arm.read(), "shoulder_pan": 30.0})
            for _ in range(30):
                arm.read()
            arm.close()  # borrower closing must not disconnect the hub
            with self.assertRaises(ArmBusy):
                hub.disconnect()
        self.assertTrue(hub.connected)
        self.assertEqual(hub.mode, "idle")
        self.assertTrue(wait_for(lambda: hub.state()["joints"]["shoulder_pan"] > 25))

    def test_cannot_borrow_while_calibrating(self):
        hub = self.make()
        hub.connect()
        hub.mode = "calibrating"
        with self.assertRaises(ArmBusy):
            with hub.borrow():
                pass
        hub.mode = "idle"

    def test_open_failure(self):
        def fail():
            raise OSError("port busy")

        hub = ArmHub(fail, "real", lambda: MODEL)
        with self.assertRaises(ArmBusy):
            hub.connect()
        self.assertIn("port busy", hub.state()["error"])

    def test_connect_applies_tuning_and_holds_with_gravity(self):
        tuned, written = [], []

        class TunedArm(FakeArm):
            def tune(self, p_gain=None, accel=None):
                tuned.append((p_gain, accel))

            def write(self, pose):
                written.append(dict(pose))
                super().write(pose)

        arm = TunedArm(gain=0.0)  # never drifts toward its target, so reads stay exact
        hub = ArmHub(lambda: arm, "real", lambda: MODEL, poll_s=0.01, tune={"p_gain": 31, "accel": 254},
                     gravity_factory=lambda: (lambda pose: {"shoulder_lift": 2.0, "elbow_flex": -1.0}))
        hub.connect()
        self.addCleanup(hub.close)
        self.assertEqual(tuned, [(31, 254)])
        hub.set_torque(False)
        hub.set_torque(True)  # hold: measured pose + gravity feed-forward
        pose = arm.read()
        self.assertAlmostEqual(written[-1]["shoulder_lift"] - pose["shoulder_lift"], 2.0, delta=1e-9)
        self.assertAlmostEqual(written[-1]["elbow_flex"] - pose["elbow_flex"], -1.0, delta=1e-9)
        sim = ArmHub(lambda: FakeArm(), "sim", lambda: MODEL)
        self.assertIsNone(sim.gravity)
        self.assertEqual(sim.hold_target({"shoulder_lift": 5.0}), {"shoulder_lift": 5.0})

    def test_no_kinematics_degrades_to_joints(self):
        def broken():
            raise ImportError("no placo")

        hub = ArmHub(lambda: FakeArm(), "sim", broken, poll_s=0.01)
        hub.connect()
        self.addCleanup(hub.close)
        self.assertTrue(wait_for(lambda: hub.state()["joints"] is not None))
        self.assertIsNone(hub.state()["frames"])


class SessionTests(unittest.TestCase):
    def test_labels_and_live_fit(self):
        with tempfile.TemporaryDirectory() as d:
            cfg = load_config(temp_config(Path(d)))
            layout = cfg.tray.layout()
            self.assertEqual([corner_label(layout, p) for p in cfg.tray.reference_points],
                             ["Bottom-left", "Top-left", "Bottom-right", "Top-right"])
            sess = CalibrationSession(cfg.tray, pan_xy=np.array([0.0388, 0.0]))
            self.assertEqual(sess.to_dict()["mode"], "corners")
            self.assertEqual([r["label"] for r in sess.to_dict()["reference_points"]],
                             ["Bottom-left corner", "Top-left corner", "Bottom-right corner", "Top-right corner"])
            sess.set_mode("pockets")
            self.assertIsNone(sess.to_dict()["fit"])
            R, t = tilted_pose()
            for pid, xy in test_tray.CalibrationTests.REFS[:3]:
                sess.capture(pid, R @ (np.array([*xy, 0]) * 0.001) + t)
            d3 = sess.to_dict()
            self.assertTrue(d3["fit"]["ok"])
            self.assertEqual([r["captured"] for r in d3["reference_points"]], [True, True, True, False])
            self.assertIn("aa", d3["fit"]["grip_angles"])
            sess.capture("glue_4", R @ np.array([0.1143, 0.04572, 0]) + t + [0.02, 0, 0])  # bad extra point
            bad = sess.to_dict()["fit"]
            self.assertFalse(bad["ok"])
            self.assertIn("glue_4", bad["error"])
            sess.remove("glue_4")
            sess.save()
            self.assertTrue(cfg.tray.calibration_file.exists())
            self.assertIsNotNone(sess.to_dict()["saved"])


class CalibrationApiTests(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.cfg = load_config(temp_config(Path(self._tmp.name)))
        layout = self.cfg.tray.layout()
        R, t = tilted_pose()
        self.refs = self.cfg.tray.reference_points
        truth = {i: R @ (np.array([layout.pocket(p).x, layout.pocket(p).y, 0]) * 0.001) + t
                 for i, p in enumerate(self.refs)}
        self.arm = FakeArm()
        self.hub = ArmHub(lambda: self.arm, "sim", lambda: FakeTipModel(truth), poll_s=0.01)
        shelf = Shelf(self.cfg)
        self.store = OrderStore(":memory:")
        catalog = Catalog(self.cfg.catalog, max_items=1, available=shelf.available)
        self.http = TestClient(create_server(self.store, catalog, shelf=shelf, hub=self.hub, auto_dispatch=True))

    def tearDown(self):
        if self.hub.connected:
            self.hub.mode = "idle"
            self.hub.close()
        self._tmp.cleanup()

    def put_tip_at(self, i):
        self.arm.pos = self.arm.target = {**self.arm.pos, "shoulder_pan": float(i)}

    def test_full_flow(self):
        self.assertEqual(self.http.post("/api/calibration/start", json={}).status_code, 400)  # no confirm
        self.assertEqual(self.http.post("/api/calibration/capture", json={"pocket": "aa_1"}).status_code, 409)
        r = self.http.post("/api/calibration/start", json={"confirm_hold": True}).json()
        self.assertTrue(r["active"])
        self.assertFalse(self.arm.torque)
        self.assertEqual(self.http.get("/api/arm").json()["mode"], "calibrating")
        self.store.create("555-0100", ["aa"], "Sam")
        self.assertIsNone(self.http.post("/robot/claim").json()["order"])  # no packing while motors are off
        for i, pid in enumerate(self.refs):
            self.put_tip_at(i)
            r = self.http.post("/api/calibration/capture", json={"pocket": pid}).json()
        self.assertTrue(r["fit"]["ok"])
        self.assertLess(r["fit"]["rms_mm"], 0.01)
        self.assertEqual(self.http.post("/api/calibration/capture", json={"pocket": "zz_9"}).status_code, 404)
        r = self.http.post("/api/calibration/save").json()
        self.assertIsNotNone(r["saved"])
        tray = self.http.get("/api/tray").json()
        self.assertIsNotNone(tray["calibration"])
        self.assertIn("robot_m", tray["pockets"][0])
        self.assertEqual(self.http.post("/api/calibration/finish", json={}).status_code, 400)
        r = self.http.post("/api/calibration/finish", json={"confirm_hold": True}).json()
        self.assertFalse(r["active"])
        self.assertTrue(self.arm.torque)
        self.assertEqual(self.http.get("/api/arm").json()["mode"], "idle")

    def test_corners_mode_over_api(self):
        layout = self.cfg.tray.layout()
        R, t = tilted_pose()
        corners = ["corner_bl", "corner_tl", "corner_br", "corner_tr"]
        truth = {i: R @ (np.array([*layout.point(c), 0]) * 0.001) + t for i, c in enumerate(corners)}
        self.hub._model = FakeTipModel(truth)
        r = self.http.post("/api/calibration/start", json={"confirm_hold": True}).json()
        self.assertEqual((r["mode"], [p["pocket"] for p in r["reference_points"]]), ("corners", corners))
        for i, c in enumerate(corners):
            self.put_tip_at(i)
            r = self.http.post("/api/calibration/capture", json={"pocket": c}).json()
        self.assertTrue(r["fit"]["ok"])
        self.assertEqual(self.http.post("/api/calibration/mode", json={"mode": "bogus"}).status_code, 400)
        r = self.http.post("/api/calibration/mode", json={"mode": "pockets"}).json()
        self.assertEqual(r["reference_points"][0]["pocket"], "aa_1")
        self.assertEqual(len(r["points"]), 4)  # captured corners are kept
        tray = self.http.get("/api/tray").json()
        self.assertEqual([c["id"] for c in tray["corners"]], ["corner_bl", "corner_tl", "corner_br", "corner_tr"])

    def test_save_refused_without_good_fit(self):
        self.http.post("/api/calibration/start", json={"confirm_hold": True})
        self.put_tip_at(0)
        self.http.post("/api/calibration/capture", json={"pocket": self.refs[0]})
        self.assertEqual(self.http.post("/api/calibration/save").status_code, 409)
        self.http.delete(f"/api/calibration/points/{self.refs[0]}")
        self.assertEqual(self.http.get("/api/calibration").json()["points"], [])

    def test_arm_endpoints_without_hub(self):
        r = self.http.post("/api/arm/connect").json()
        self.assertTrue(r["connected"])
        http = TestClient(create_server(OrderStore(":memory:"), Catalog({}, 1)))
        self.assertEqual(http.get("/api/arm").json()["connected"], False)
        self.assertEqual(http.post("/api/arm/connect").status_code, 503)

    def test_preview_poses_need_no_arm(self):
        remote = ArmHub(None, "none", lambda: MODEL)
        http = TestClient(create_server(OrderStore(":memory:"), Catalog({}, 1), hub=remote))
        names = [p["name"] for p in http.get("/api/arm/preview").json()["poses"]]
        self.assertIn("over_tray", names)
        st = http.get("/api/arm/preview/over_tray").json()
        self.assertEqual((st["source"], st["connected"]), ("preview", False))
        self.assertEqual(list(st["frames"]), ArmModel.LINKS)
        self.assertAlmostEqual(st["tip"][0], 0.24, delta=0.01)  # out over the tray, pointing down
        self.assertEqual(http.get("/api/arm/preview/nope").status_code, 404)


class GotoTests(unittest.TestCase):
    """Go-to preview: plan to a tray point + stand-off along the normal, run it on a FakeArm, stop it."""

    def setUp(self):
        from packing.tray.calibration import calibrate

        self._tmp = tempfile.TemporaryDirectory()
        self.cfg = load_config(temp_config(Path(self._tmp.name)))
        R, t = tilted_pose(40, 0.22)
        self.cal = calibrate([(p, xy, R @ (np.array([*xy, 0]) * 0.001) + t) for p, xy in test_tray.CalibrationTests.REFS])
        self.cal.save(self.cfg.tray.calibration_file)
        self.arm = FakeArm(gain=0.9)
        self.hub = ArmHub(lambda: self.arm, "sim", lambda: MODEL, poll_s=0.01)
        self.hub.connect()

    def tearDown(self):
        self.hub.mode = "idle"
        self.hub.close()
        self._tmp.cleanup()

    def test_plan_reaches_standoff_along_normal(self):
        from packing.tray.goto import plan_goto

        xy = self.cfg.tray.layout().point("corner_tr")
        c = self.cal.to_robot(*xy)
        plan = plan_goto(MODEL, "corner_tr", c, self.cal.normal, dict.fromkeys(MODEL.to_units(np.zeros(5)), 0.0))
        end = MODEL.fk(plan.path[-1])
        self.assertAlmostEqual(float((end[:3, 3] - c) @ self.cal.normal) * 1000, 12.7, delta=0.5)
        self.assertLess(np.linalg.norm(end[:3, 3] - (c + self.cal.normal * 0.0127)) * 1000, 2.0)
        self.assertLess(plan.tilt_deg, 25)  # as close to the normal as the 5-DOF arm allows

    def test_run_and_stop(self):
        from packing.shop.goto_runner import GotoRunner
        from packing.tray.goto import plan_goto

        xy = self.cfg.tray.layout().point("corner_bl")
        c = self.cal.to_robot(*xy)
        plan = plan_goto(MODEL, "corner_bl", c, self.cal.normal, self.hub.state()["joints"] or self.arm.read())
        events = []
        runner = GotoRunner(self.hub, self.cfg, lambda k, t, **d: events.append(t), sleep=lambda s: None)
        runner.start(plan)
        runner.join(10)
        self.assertEqual(runner.last["status"], "done", runner.last)
        tip = MODEL.tip(self.arm.read())
        self.assertLess(np.linalg.norm(tip - (c + self.cal.normal * 0.0127)) * 1000, 3.0)
        self.assertEqual(self.hub.mode, "idle")
        # stop: a sleep that presses Stop after a few ticks
        ticks = [0]

        def sleep(_s):
            ticks[0] += 1
            if ticks[0] == 5:
                runner.stop()

        self.arm.pos = self.arm.target = dict.fromkeys(self.arm.pos, 0.0)
        runner.sleep = sleep
        plan2 = plan_goto(MODEL, "corner_tr", self.cal.to_robot(*self.cfg.tray.layout().point("corner_tr")),
                          self.cal.normal, self.arm.read())
        runner.start(plan2)
        runner.join(10)
        self.assertEqual(runner.last["status"], "stopped")
        self.assertTrue(any("stopped by the user" in e for e in events))

    def test_api_dry_run_needs_confirm_and_turns_motors_on_when_calibrating(self):
        shelf = Shelf(self.cfg)
        http = TestClient(create_server(OrderStore(":memory:"), Catalog(self.cfg.catalog, 1), shelf=shelf, hub=self.hub))
        self.assertTrue(wait_for(lambda: self.hub.state()["joints"] is not None))
        r = http.post("/api/arm/goto", json={"point": "corner_tl"}).json()
        self.assertFalse(r["started"])
        self.assertEqual(r["calibration"], "saved calibration")
        self.assertFalse(r["motors_will_turn_on"])
        self.assertEqual(r["plan"]["standoff_mm"], 12.7)
        self.assertEqual(http.post("/api/arm/goto", json={"point": "nope"}).status_code, 404)
        # during calibration the motors are off: the dry run says they'll turn on; capture refuses while on
        http.post("/api/calibration/start", json={"confirm_hold": True})
        r = http.post("/api/arm/goto", json={"point": "corner_tl"}).json()
        self.assertTrue(r["motors_will_turn_on"])
        self.hub.set_torque(True)
        self.assertEqual(http.post("/api/calibration/capture", json={"pocket": "corner_bl"}).status_code, 409)
        self.assertEqual(http.post("/api/arm/release", json={}).status_code, 400)
        self.assertFalse(http.post("/api/arm/release", json={"confirm_hold": True}).json()["torque"])


if __name__ == "__main__":
    unittest.main()
