"""Auto pack toggle (PUT /api/auto-pack) and a robot agent that never dies. Sim only."""

import tempfile
import unittest
from pathlib import Path

from fastapi.testclient import TestClient

from packing.boxes.geometry import box_frame
from packing.boxes.store import BoxStore
from packing.config import load_config
from packing.service.orders import OrderStore
from packing.service.vapi import Catalog
from packing.shop.robot_agent import run
from packing.shop.server import create_server
from packing.shop.shelf import Shelf
from packing.tests.test_arm_view import MODEL
from packing.tests.test_boxes import tool_in_box
from packing.tests.test_pack import SUPPLY
from packing.tests.test_tray import temp_config


class AutoPackTests(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.cfg = load_config(temp_config(Path(self._tmp.name)))
        self.shelf = Shelf(self.cfg)
        for p in ("aa_1", "aa_2", "glue_1"):
            self.shelf.set_slot(p, True)
        self.store = OrderStore(":memory:")
        self.boxes = BoxStore(self.store)
        catalog = Catalog(self.cfg.catalog, 10, available=self.shelf.available, stock=self.shelf.stock)
        self.http = TestClient(create_server(self.store, catalog, shelf=self.shelf, box_packing=True))

    def tearDown(self):
        self._tmp.cleanup()

    def order(self, items):
        return self.store.create("", items, "Sam", stock=self.shelf.stock)[0]

    def add_box(self, dy=0.0):
        return self.boxes.add(box_frame((SUPPLY[0], SUPPLY[1] + dy), 0.0, MODEL.pan_xy), tool_in_box(), {}, {})

    def claim(self):
        o = self.http.post("/robot/claim").json()["order"]
        return o and o["id"]

    def auto(self, on):
        return self.http.put("/api/auto-pack", json={"enabled": on}).json()

    def test_off_does_nothing(self):
        self.add_box()
        self.order(["aa"])
        self.assertFalse(self.http.get("/api/auto-pack").json()["enabled"])
        self.assertIsNone(self.claim())

    def test_on_packs_the_newest_order_with_all_its_stock(self):
        self.add_box()
        a = self.order(["aa"])
        b = self.order(["glue"])
        self.shelf.set_slot("glue_2", True)
        c = self.order(["glue"])  # gets glue_2 (glue_1 went to b) ...
        self.shelf.set_slot("glue_2", False)  # ... then it's marked empty: c goes short
        self.store.release_slot("glue_2", self.shelf.stock())
        self.assertIsNone(self.store.get(c.id).slots[0])
        st = self.auto(True)
        self.assertEqual((st["enabled"], st["next_order"]), (True, b.id))
        self.assertEqual(self.claim(), b.id)
        self.assertEqual(self.http.get("/api/state").json()["auto_pack"]["enabled"], True)
        self.assertEqual(self.store.get(a.id).status, "queued")

    def test_actioned_orders_go_first(self):
        self.add_box()
        a = self.order(["aa"])
        self.order(["glue"])
        self.http.post(f"/api/orders/{a.id}/action")
        self.auto(True)
        self.assertEqual(self.claim(), a.id)

    def test_waits_for_a_box(self):
        o = self.order(["aa"])
        st = self.auto(True)
        self.assertIn("no empty box", st["waiting"])
        self.assertIsNone(self.claim())
        b = self.add_box()
        other = self.add_box(0.13)
        self.boxes.update(other.id, location="pack")
        self.assertIn(f"box {other.id} is still at the pack spot", self.auto(True)["waiting"])
        self.assertIsNone(self.claim())
        self.boxes.remove(other.id)
        self.assertEqual(self.claim(), o.id)
        self.assertEqual(self.boxes.get(b.id).state, "available")  # the packer takes it, not the claim

    def test_refused_orders_wait_for_action(self):
        self.add_box()
        o = self.order(["aa"])
        self.auto(True)
        self.assertEqual(self.claim(), o.id)
        self.http.post(f"/robot/orders/{o.id}/requeue", json={"error": "can't plan"})
        self.assertIsNone(self.claim())
        self.assertIn("refused", self.auto(True)["waiting"])
        self.http.post(f"/api/orders/{o.id}/action")
        self.assertEqual(self.claim(), o.id)

    def test_a_failure_turns_auto_pack_off(self):
        self.add_box()
        a = self.order(["aa"])
        b = self.order(["aa"])
        self.auto(True)
        self.assertEqual(self.claim(), b.id)  # newest first
        self.http.post(f"/robot/orders/{b.id}/failed", json={"error": "gripper missed"})
        self.assertFalse(self.http.get("/api/auto-pack").json()["enabled"])
        self.assertIsNone(self.claim())
        self.assertEqual(self.store.get(a.id).status, "queued")


class AgentKeepsGoingTests(unittest.TestCase):
    def test_agent_keeps_polling_after_a_failure(self):
        store = OrderStore(":memory:")
        http = TestClient(create_server(store, Catalog({"A": "Widget"}, 2), auto_dispatch=True))
        store.create("", ["A"], "Sam")
        store.create("", ["A"], "Al")

        class Packer:
            calls = 0

            def pack(self, order):
                Packer.calls += 1
                if Packer.calls == 1:
                    raise RuntimeError("gripper missed")
                return "BOX-1"

        sleeps = []

        def sleep(s):  # stop the endless loop after a few polls
            sleeps.append(s)
            if len(sleeps) > 5:
                raise KeyboardInterrupt
        with self.assertRaises(KeyboardInterrupt):
            run("http://test", Packer(), http=http, sleep=sleep)
        self.assertEqual(Packer.calls, 1)  # auto pack went off after the failure: nothing more claimed
        self.assertEqual(store.get(2).status, "failed")  # newest first
        self.assertEqual(store.get(1).status, "queued")
        self.assertEqual(http.get("/api/state").json()["robot"]["state"], "idle")  # still checking in


if __name__ == "__main__":
    unittest.main()
