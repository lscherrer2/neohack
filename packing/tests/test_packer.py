"""TrayPacker bookkeeping (ROADMAP "Next steps" §2, §4, §5): one-way box flow, parts marked used per item, a missed
grip marks the pocket empty and stops the order, refusals before the arm moves. Sim only (FakeArm)."""

import tempfile
import unittest
from pathlib import Path

from fastapi.testclient import TestClient

from packing.boxes.geometry import box_frame
from packing.boxes.packer import NotStarted, PocketEmpty, TrayPacker
from packing.boxes.store import BoxStore
from packing.config import load_config
from packing.hardware.arm import FakeArm
from packing.service.orders import OrderStore
from packing.service.vapi import Catalog
from packing.shop import robot_agent
from packing.shop.arm_hub import ArmHub
from packing.shop.server import create_server
from packing.shop.shelf import Shelf
from packing.tests.test_arm_view import MODEL
from packing.tests.test_boxes import tool_in_box
from packing.tests.test_pack import SUPPLY, left_tray
from packing.tests.test_tray import temp_config

PAN = MODEL.pan_xy


class TrayPackerFlowTests(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.cfg = load_config(temp_config(Path(self._tmp.name)))
        left_tray(self.cfg)
        self.shelf = Shelf(self.cfg)
        for p in ("aa_1", "aa_2", "glue_3"):
            self.shelf.set_slot(p, True)
        self.store = OrderStore(":memory:")
        self.boxes = BoxStore(self.store)
        self.box = self.boxes.add(box_frame(SUPPLY, 0.0, PAN), tool_in_box(), {}, {})
        self.arm = FakeArm(gain=0.9)
        self.arm.gripper_block = 5.0  # something between the jaws at every grip (tab, parts)
        self.hub = ArmHub(lambda: self.arm, "sim", lambda: MODEL, poll_s=0.01)
        self.hub.connect()
        self.events: list[tuple[str, str]] = []

    def tearDown(self):
        self.hub.mode = "idle"
        self.hub.close()
        self._tmp.cleanup()

    def packer(self):
        return TrayPacker(self.hub, self.cfg, self.boxes, self.shelf, lambda k, t, **d: self.events.append((k, t)),
                          sleep=lambda s: None)

    def order(self, items):
        o, _ = self.store.create("", items, "Sam", stock=self.shelf.stock)
        return self.store.claim_next("queued")

    def test_parts_are_marked_used_one_by_one(self):
        o = self.order(["aa", "glue"])
        p = self.packer()
        seen = []  # (slot, box location, full slots) at each consume
        real = self.shelf.consume_slots

        def consume(slots):
            real(slots)
            seen.append((slots[0], self.boxes.get(self.box.id).location, self.shelf.full_slots()))
        self.shelf.consume_slots = consume
        p.pack(o)
        self.assertEqual([s[0] for s in seen], ["aa_1", "glue_3"])
        self.assertTrue(all(loc == "pack" for _, loc, _ in seen))  # during the order, not after it
        self.assertNotIn("aa_1", seen[0][2])
        self.assertIn("glue_3", seen[0][2])  # the second part is still in the tray after the first is taken
        items = [t for _, t in self.events if " item " in t]
        self.assertEqual(len(items), 2)
        self.assertIn("item 1/2: aa from aa_1", items[0])
        b = self.boxes.get(self.box.id)
        self.assertEqual((b.state, b.location), ("packed", "output"))

    def test_missed_grip_marks_the_pocket_empty_and_stops(self):
        o = self.order(["aa"])
        self.arm.gripper_block = None  # nothing in the jaws: the box tab plate grip has no width check, the part's does
        with self.assertRaises(PocketEmpty) as cm:
            self.packer().pack(o)
        self.assertEqual(cm.exception.pocket, "aa_1")
        self.assertNotIn("aa_1", self.shelf.full_slots())
        self.assertEqual(self.store.get(o.id).slots, ["aa_2"])  # its reservation moved to the next unit
        self.assertEqual(self.boxes.get(self.box.id).location, "pack")  # left as it is for a person to sort out
        self.assertTrue(any(k == "error" and "aa_1" in t for k, t in self.events))
        self.assertEqual(self.hub.mode, "idle")

    def test_pack_spot_taken_refuses_before_moving(self):
        other = self.boxes.add(box_frame((SUPPLY[0], SUPPLY[1] + 0.13), 0.0, PAN), tool_in_box(), {}, {})
        self.boxes.update(other.id, location="pack")
        o = self.order(["aa"])
        before = dict(self.arm.read())
        with self.assertRaises(NotStarted) as cm:
            self.packer().pack(o)
        self.assertIn(f"box {other.id} is still at the pack spot", str(cm.exception))
        self.assertEqual(self.boxes.get(self.box.id).state, "available")  # no box kept
        self.assertEqual(self.arm.read(), before)

    def test_unplannable_gives_the_box_back(self):
        self.cfg.tray.calibration_file.unlink()  # no tray calibration: can't plan
        o = self.order(["aa"])
        with self.assertRaises(NotStarted):
            self.packer().pack(o)
        b = self.boxes.get(self.box.id)
        self.assertEqual((b.state, b.order_id), ("available", None))

    def test_robot_agent_requeues_a_refused_order(self):
        self.boxes.remove(self.box.id)  # no box at all
        catalog = Catalog(self.cfg.catalog, 10, available=self.shelf.available, stock=self.shelf.stock)
        http = TestClient(create_server(self.store, catalog, shelf=self.shelf, hub=self.hub))
        o, _ = self.store.create("", ["aa"], "Sam", stock=self.shelf.stock)
        self.store.action(o.id)
        rc = robot_agent.run("http://test", self.packer(), once=True, http=http, sleep=lambda s: None)
        self.assertEqual(rc, 0)  # keeps going: nothing moved
        o = self.store.get(o.id)
        self.assertEqual(o.status, "queued")
        self.assertIn("no empty box", o.error)
        self.assertEqual(o.slots, ["aa_1"])  # still reserved


class BoxFlowApiTests(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.cfg = load_config(temp_config(Path(self._tmp.name)))
        self.shelf = Shelf(self.cfg)
        self.store = OrderStore(":memory:")
        self.boxes = BoxStore(self.store)
        self.box = self.boxes.add(box_frame(SUPPLY, 0.0, PAN), tool_in_box(), {}, {})
        self.hub = ArmHub(lambda: FakeArm(), "sim", lambda: MODEL, poll_s=0.01)
        self.hub.connect()
        catalog = Catalog(self.cfg.catalog, 10, available=self.shelf.available, stock=self.shelf.stock)
        self.http = TestClient(create_server(self.store, catalog, shelf=self.shelf, hub=self.hub))

    def tearDown(self):
        self.hub.close()
        self._tmp.cleanup()

    def move(self, to):
        return self.http.post("/api/arm/box-move", json={"box_id": self.box.id, "to": to})

    def test_boxes_only_move_forward(self):
        self.assertEqual(self.move("supply").status_code, 400)
        self.boxes.update(self.box.id, location="output", state="packed")
        r = self.move("pack")
        self.assertEqual(r.status_code, 409)
        self.assertIn("left → right", r.json()["detail"])

    def test_pack_spot_taken_refuses_a_box_move(self):
        other = self.boxes.add(box_frame((SUPPLY[0], SUPPLY[1] + 0.13), 0.0, PAN), tool_in_box(), {}, {})
        self.boxes.update(other.id, location="pack")
        r = self.move("pack")
        self.assertEqual(r.status_code, 409)
        self.assertIn("still at the pack spot", r.json()["detail"])

    def test_reset_is_gone_and_picked_up_deletes(self):
        self.assertIn(self.http.post(f"/api/boxes/{self.box.id}/reset").status_code, (404, 405))
        self.boxes.update(self.box.id, location="output", state="packed")
        r = self.http.delete(f"/api/boxes/{self.box.id}")
        self.assertEqual(r.status_code, 200)
        self.assertEqual(r.json()["boxes"], [])
        self.assertTrue(any("picked up" in str(e) for e in self.http.get("/api/events").json()))

    def test_box_of_a_failed_order_can_be_removed(self):
        o, _ = self.store.create("", [], "Sam")
        self.boxes.take(o.id)
        self.store.update(o.id, "failed", error="x")
        self.assertEqual(self.http.delete(f"/api/boxes/{self.box.id}").status_code, 200)


if __name__ == "__main__":
    unittest.main()
