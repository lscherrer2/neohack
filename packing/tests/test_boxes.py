"""Packing boxes: geometry + capture math, box records, box-move planning (lift/carry/land checks, collisions),
and the API end to end on a FakeArm posed at a tab grasp. Sim only."""

import tempfile
import time
import unittest
from pathlib import Path

import numpy as np
from fastapi.testclient import TestClient

from packing.boxes.geometry import (BOX_H, HALF, TAB_T, CaptureError, GripCfg, box_frame, box_from_grasp, box_tilt_deg,
                                    footprint, mirror, overlaps)
from packing.boxes.move import box_pose, plan_box_move
from packing.boxes.store import Box, BoxError, BoxStore
from packing.config import load_config
from packing.hardware.arm import FakeArm
from packing.motion.kinematics import PathError
from packing.motion.program import Grip, LineMove
from packing.service.orders import OrderStore
from packing.service.vapi import Catalog
from packing.shop.arm_hub import ArmHub
from packing.shop.goto_runner import GotoRunner
from packing.shop.server import create_server
from packing.shop.shelf import Shelf
from packing.tests.test_arm_view import MODEL
from packing.tests.test_tray import temp_config
from packing.tray.pick import AxisIK

PAN = MODEL.pan_xy
HOME = dict.fromkeys(["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"], 0.0)


def tool_in_box(g=GripCfg()):
    """Gripper level, pointing along the tab toward the box (+y), jaws closing up/down; fixed jaw UNDER the plate
    (tool x down — wrist roll near 0; jaw-on-top needs roll ~180°, past its limit), so the tool point is on the
    plate's bottom face."""
    T = np.eye(4)
    T[:3, 0], T[:3, 2] = [0, 0, -1], [0, 1, 0]
    T[:3, 1] = np.cross(T[:3, 2], T[:3, 0])
    T[:3, 3] = [0.0, -(HALF + g.from_wall_mm / 1000), BOX_H - TAB_T]
    return T


def make_box(cx, cy, bid=1):
    return Box(bid, "available", "supply", None, box_frame((cx, cy), 0.0, PAN), tool_in_box(), {}, {}, "", "")


class GeometryTests(unittest.TestCase):
    def test_capture_recovers_box_and_tab_faces_swivel(self):
        T_box = box_frame((0.32, 0.16), 0.0, PAN)
        got, rep = box_from_grasp(T_box @ tool_in_box(), PAN)
        np.testing.assert_allclose(got, T_box, atol=1e-9)
        out = got[:3, 3][:2] - PAN
        self.assertGreater(float(got[:2, 1] @ out), 0.99 * np.linalg.norm(out))  # +y radially outward
        self.assertEqual(rep["tilt_deg"], 0.0)
        self.assertEqual(box_tilt_deg(T_box @ tool_in_box(), tool_in_box()), 0.0)

    def test_capture_refuses_tilted_or_floating(self):
        T = box_frame((0.32, 0.16), 0.0, PAN) @ tool_in_box()
        down = T.copy()
        down[:3, :3] = np.column_stack([[1, 0, 0], [0, -1, 0], [0, 0, -1]])  # pointing straight down
        with self.assertRaises(CaptureError):
            box_from_grasp(down, PAN)
        backwards = T.copy()
        backwards[:3, 2] *= -1  # level but pointing back at the arm
        backwards[:3, 1] *= -1
        with self.assertRaises(CaptureError):
            box_from_grasp(backwards, PAN)
        high = T.copy()
        high[2, 3] += 0.05
        with self.assertRaises(CaptureError):
            box_from_grasp(high, PAN)

    def test_mirror_and_overlap(self):
        a = box_frame((0.22, 0.17), 0.0, PAN)
        m = mirror(a, PAN)
        self.assertAlmostEqual(m[1, 3], -0.17)
        self.assertFalse(overlaps(footprint(a), footprint(m)))
        self.assertTrue(overlaps(footprint(a), footprint(box_frame((0.25, 0.18), 0.0, PAN))))
        self.assertFalse(overlaps(footprint(a), footprint(box_frame((0.10, 0.17), 0.0, PAN))))  # 12 cm apart; 10 cm: tab touches


class StoreTests(unittest.TestCase):
    def test_take_reuse_and_lifecycle(self):
        s = BoxStore(OrderStore(":memory:"))
        b1 = s.add(np.eye(4), np.eye(4), {}, {})
        b2 = s.add(np.eye(4), np.eye(4), {}, {})
        self.assertEqual(s.take(7).id, b1.id)
        self.assertEqual(s.take(7).id, b1.id)  # same order: same box
        self.assertEqual(s.take(8).id, b2.id)
        with self.assertRaises(BoxError):
            s.take(9)
        with self.assertRaises(BoxError):
            s.remove(b1.id)  # in use
        s.update(b1.id, location="output", state="packed")
        s.remove(b1.id)
        self.assertEqual([b.id for b in s.list()], [b2.id])


class PlanTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls._tmp = tempfile.TemporaryDirectory()
        cls.cfg = load_config(temp_config(Path(cls._tmp.name)))

    @classmethod
    def tearDownClass(cls):
        cls._tmp.cleanup()

    def test_supply_pack_output_lands_exactly_level_and_square(self):
        box = make_box(0.32, 0.16)
        cur = HOME
        for src, dst in (("supply", "pack"), ("pack", "output")):
            plan = plan_box_move(MODEL, box, src, dst, cur, self.cfg)
            kinds = [type(s).__name__ for s in plan.steps]
            self.assertEqual(kinds, ["OpenGripper", "JointMove", "LineMove", "Grip", "LineMove", "CheckHolding",
                                     "JointMove", "LineMove", "OpenGripper", "LineMove"])
            come_in = plan.steps[2]  # level along the tab
            d_in = MODEL.tip(come_in.path[-1]) - MODEL.tip(come_in.path[0])
            self.assertLess(abs(d_in[2]) * 1000, 1.5)
            self.assertAlmostEqual(np.linalg.norm(d_in) * 1000, self.cfg.boxes.approach_mm, delta=1.5)
            grasp = MODEL.fk(plan.steps[2].path[-1])
            want_src = box_pose(box, src, self.cfg, PAN) @ box.tool_in_box
            self.assertLess(np.linalg.norm(grasp[:3, 3] - want_src[:3, 3]) * 1000, 1.0)
            lift = plan.steps[4]
            d = MODEL.tip(lift.path[-1]) - MODEL.tip(lift.path[0])
            np.testing.assert_allclose(d, [0, 0, self.cfg.boxes.carry_mm / 1000], atol=0.002)  # straight up
            land = MODEL.fk(plan.steps[7].path[-1])
            want = box_pose(box, dst, self.cfg, PAN) @ box.tool_in_box
            self.assertLess(np.linalg.norm(land[:3, 3] - want[:3, 3]) * 1000, 1.0)  # same spot + height
            self.assertAlmostEqual(land[2, 3], want_src[2, 3], places=3)  # set down at the picked height
            self.assertLessEqual(plan.tilt_deg, 5.0)
            self.assertLessEqual(plan.max_carry_tilt_deg, 30.0)
            cur = plan.steps[-1].path[-1]

    def test_refuses_collisions_and_tilted_landings(self):
        box = make_box(0.32, 0.16)
        blocker = ("box 2", footprint(box_frame((0.35, 0.0), 0.0, PAN)))  # sitting on the pack spot
        with self.assertRaises(PathError) as e:
            plan_box_move(MODEL, box, "supply", "pack", HOME, self.cfg, [blocker])
        self.assertIn("overlap box 2", str(e.exception))
        near = make_box(0.22, 0.17)  # tab grip point < 25 cm from the swivel: the level grip can't reach it
        with self.assertRaises(PathError):
            plan_box_move(MODEL, near, "supply", "pack", HOME, self.cfg)


class ApiTests(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.cfg = load_config(temp_config(Path(self._tmp.name)))
        self.arm = FakeArm(gain=0.9)
        self.hub = ArmHub(lambda: self.arm, "sim", lambda: MODEL, poll_s=0.01)
        self.hub.connect()
        self.store = OrderStore(":memory:")
        self.http = TestClient(create_server(self.store, Catalog(self.cfg.catalog, 10), shelf=Shelf(self.cfg),
                                             hub=self.hub))

    def tearDown(self):
        self.hub.mode = "idle"
        self.hub.close()
        self._tmp.cleanup()

    def pose_at_tab(self, cx, cy):
        """Put the (limp) fake arm where a person would hold the jaws on that box's tab."""
        T = box_frame((cx, cy), 0.0, PAN) @ tool_in_box()
        from packing.boxes.move import down_seeds

        q = min((AxisIK(MODEL).solve(s, T[:3, 3], T[:3, 2], T[:3, 0]) for s in down_seeds(MODEL, T[:3, 3])),
                key=lambda r: r[1])[0]
        self.arm.pos = self.arm.target = {**q, "gripper": 5.0}

    def test_capture_list_move_and_remove(self):
        self.pose_at_tab(0.32, 0.16)
        self.assertEqual(self.http.post("/api/boxes/capture").status_code, 409)  # motors on
        self.hub.set_torque(False)
        r = self.http.post("/api/boxes/capture")
        self.assertEqual(r.status_code, 200, r.text)
        b = r.json()["boxes"][0]
        np.testing.assert_allclose(np.array(b["pose"])[:2, 3], [0.32, 0.16], atol=0.003)
        self.assertEqual(self.http.post("/api/boxes/capture").status_code, 422)  # same box again: overlap
        self.pose_at_tab(0.32, -0.16)
        r = self.http.post("/api/boxes/capture")
        self.assertIn("LEFT", r.json()["detail"])
        # dry run, then move it to the pack spot on the fake arm
        r = self.http.post("/api/arm/box-move", json={"box_id": b["id"], "to": "pack"}).json()
        self.assertFalse(r["started"])
        self.assertTrue(r["motors_will_turn_on"])
        self.arm.gripper_block = 1.2  # the 2 mm tab plate between the jaws
        r = self.http.post("/api/arm/box-move", json={"box_id": b["id"], "to": "pack", "confirm_move": True})
        self.assertTrue(r.json()["started"], r.text)
        deadline = time.monotonic() + 60
        while (st := self.http.get("/api/arm/goto").json())["running"] and time.monotonic() < deadline:
            time.sleep(0.05)
        self.assertEqual(st["last"]["status"], "done", st)
        b2 = self.http.get("/api/boxes").json()["boxes"][0]
        self.assertEqual(b2["location"], "pack")
        np.testing.assert_allclose(np.array(b2["pose"])[:2, 3], [0.345, 0.0], atol=1e-6)
        self.assertEqual(self.http.delete(f"/api/boxes/{b['id']}").status_code, 200)
        self.assertEqual(self.http.get("/api/boxes").json()["boxes"], [])


if __name__ == "__main__":
    unittest.main()
