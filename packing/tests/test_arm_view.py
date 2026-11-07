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
from packing.tests.test_tray import CalibrationTests, temp_config, tilted_pose
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

    def test_open_failure_and_remote_telemetry(self):
        def fail():
            raise OSError("port busy")

        hub = ArmHub(fail, "real", lambda: MODEL)
        with self.assertRaises(ArmBusy):
            hub.connect()
        self.assertIn("port busy", hub.state()["error"])
        remote = ArmHub(None, "remote", lambda: MODEL)
        remote.external(dict.fromkeys(["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll",
                                       "gripper"], 0.0), True)
        st = remote.state()
        self.assertEqual((st["connected"], st["source"], st["mode"]), (True, "remote", "packing"))
        self.assertIsNotNone(st["frames"])

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
            for pid, xy in CalibrationTests.REFS[:3]:
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

    def test_arm_endpoints_and_telemetry_without_hub(self):
        r = self.http.post("/api/arm/connect").json()
        self.assertTrue(r["connected"])
        http = TestClient(create_server(OrderStore(":memory:"), Catalog({}, 1)))
        self.assertEqual(http.get("/api/arm").json()["connected"], False)
        self.assertEqual(http.post("/api/arm/connect").status_code, 503)
        self.assertEqual(http.post("/robot/telemetry", json={"joints": {"shoulder_pan": 1.0}}).status_code, 200)

    def test_preview_poses_need_no_arm(self):
        remote = ArmHub(None, "remote", lambda: MODEL)
        http = TestClient(create_server(OrderStore(":memory:"), Catalog({}, 1), hub=remote))
        names = [p["name"] for p in http.get("/api/arm/preview").json()["poses"]]
        self.assertIn("over_tray", names)
        st = http.get("/api/arm/preview/over_tray").json()
        self.assertEqual((st["source"], st["connected"]), ("preview", False))
        self.assertEqual(list(st["frames"]), ArmModel.LINKS)
        self.assertAlmostEqual(st["tip"][0], 0.24, delta=0.01)  # out over the tray, pointing down
        self.assertEqual(http.get("/api/arm/preview/nope").status_code, 404)

    def test_remote_telemetry_reaches_view(self):
        remote = ArmHub(None, "remote", lambda: MODEL)
        http = TestClient(create_server(OrderStore(":memory:"), Catalog({}, 1), hub=remote))
        joints = {"shoulder_pan": 10.0, "shoulder_lift": 0.0, "elbow_flex": 0.0, "wrist_flex": 0.0,
                  "wrist_roll": 0.0, "gripper": 20.0}
        http.post("/robot/telemetry", json={"joints": joints, "torque": True})
        st = http.get("/api/arm").json()
        self.assertEqual((st["source"], st["joints"]["shoulder_pan"]), ("remote", 10.0))
        self.assertEqual(http.post("/api/arm/connect").status_code, 409)


if __name__ == "__main__":
    unittest.main()
