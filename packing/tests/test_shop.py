"""Slim shop stack tests (no network, no hardware, no API calls)."""

import asyncio
import unittest

from fastapi.testclient import TestClient
from mcp import Client

from packing.service.orders import OrderStore
from packing.service.vapi import Catalog
from packing.service.worker import SimPacker
from packing.shop.robot_agent import run
from packing.shop.server import build_mcp, create_server
from packing.shop.tools import ShopTools


class ShopTests(unittest.TestCase):
    def setUp(self):
        self.store = OrderStore(":memory:")
        self.catalog = Catalog({"A": "widget", "B": "gadget"}, max_items=2, available=lambda: {"A": 3, "B": 1})
        self.tools = ShopTools(self.store, self.catalog)

    def mcp(self, name, args=None):
        async def go():
            async with Client(build_mcp(self.tools)) as c:
                res = await c.call_tool(name, args or {})
                return res.content[0].text

        return asyncio.run(go())

    def test_mcp_tools_listed(self):
        async def go():
            async with Client(build_mcp(self.tools)) as c:
                return sorted(t.name for t in (await c.list_tools()).tools)

        self.assertEqual(asyncio.run(go()), ["cancel_order", "check_order", "list_products", "place_order"])

    def test_order_flow_over_mcp(self):
        self.assertIn("widget (3 in stock)", self.mcp("list_products"))
        r = self.mcp("place_order", {"items": [{"product": "widget"}, {"product": "B"}], "customer_name": "Sam"})
        self.assertIn("Order number 1 for Sam", r)
        self.assertIn("has been received", self.mcp("check_order", {"order_id": 1}))
        self.assertIn("only 0 left", self.mcp("place_order", {"items": [{"product": "gadget"}], "customer_name": "Al"}))
        self.assertIn("cancelled", self.mcp("cancel_order", {"order_id": 1}))

    def test_problems_are_answers(self):
        self.assertIn("don't carry", self.mcp("place_order", {"items": [{"product": "sprocket"}], "customer_name": "Sam"}))
        self.assertIn("up to 2 items", self.mcp("place_order", {"items": [{"product": "A", "quantity": 3}], "customer_name": "Sam"}))
        self.assertIn("couldn't find", self.mcp("check_order", {"order_id": 99}))

    def test_robot_agent_packs_via_api(self):
        http = TestClient(create_server(self.store, self.catalog, auto_dispatch=True))
        self.tools.place_order(["A"], "Sam")
        self.assertEqual(run("http://test", SimPacker(0), once=True, http=http), 0)
        o = self.store.get(1)
        self.assertEqual((o.status, o.box_id), ("packed", "SIM-BOX-1"))
        self.assertIn("ready for pickup", self.tools.check_order(1))

    def test_claim_resumes_in_progress_order(self):
        http = TestClient(create_server(self.store, self.catalog, auto_dispatch=True))
        self.tools.place_order(["A"], "Sam")
        self.tools.place_order(["B"], "Al")
        first = http.post("/robot/claim").json()["order"]["id"]
        again = http.post("/robot/claim").json()["order"]["id"]  # agent restarted mid-pack
        self.assertEqual(first, again)

    def test_failure_reported_and_agent_stops(self):
        class Boom:
            def pack(self, order):
                raise RuntimeError("gripper missed")

        http = TestClient(create_server(self.store, self.catalog, auto_dispatch=True))
        self.tools.place_order(["A"], "Sam")
        self.assertEqual(run("http://test", Boom(), once=True, http=http), 1)
        self.assertEqual((self.store.get(1).status, self.store.get(1).error), ("failed", "gripper missed"))

    def test_manual_mode_waits_for_action(self):
        http = TestClient(create_server(self.store, self.catalog))  # default: manual
        self.tools.place_order(["A"], "Sam")
        self.assertIsNone(http.post("/robot/claim").json()["order"])  # not actioned yet
        self.assertEqual(http.post("/api/orders/1/action").json()["status"], "ready")
        self.assertEqual(http.post("/api/orders/1/action").status_code, 409)  # only received orders
        self.assertEqual(http.post("/robot/claim").json()["order"]["id"], 1)

    def test_delete(self):
        http = TestClient(create_server(self.store, self.catalog))
        self.tools.place_order(["B"], "Sam")
        self.assertIn("only 0 left", self.tools.list_products() + self.mcp(
            "place_order", {"items": [{"product": "B"}], "customer_name": "Al"}))  # reserved
        self.assertEqual(http.delete("/api/orders/1").status_code, 200)
        self.assertIsNone(self.store.get(1))
        self.assertIn("gadget (1 in stock)", self.tools.list_products())  # reservation released
        self.tools.place_order(["B"], "Al")
        http.post("/api/orders/2/action")
        http.post("/robot/claim")
        self.assertEqual(http.delete("/api/orders/2").status_code, 409)  # being packed

    def test_done_requires_packing_state(self):
        http = TestClient(create_server(self.store, self.catalog))
        self.tools.place_order(["A"], "Sam")
        self.assertEqual(http.post("/robot/orders/1/done", json={}).status_code, 409)


if __name__ == "__main__":
    unittest.main()


class DashboardTests(unittest.TestCase):
    def setUp(self):
        from packing.shop.events import EventLog

        self.store = OrderStore(":memory:")
        self.catalog = Catalog({"A": "widget"}, max_items=2, available=lambda: {"A": 5})
        self.events = EventLog()
        self.http = TestClient(create_server(self.store, self.catalog, self.events, auto_dispatch=True))

    def test_events_from_tools_and_robot(self):
        async def place():
            async with Client(build_mcp(ShopTools(self.store, self.catalog), self.events)) as c:
                await c.call_tool("place_order", {"items": [{"product": "widget"}], "customer_name": "Sam"})
                await c.call_tool("place_order", {"items": [{"product": "sprocket"}], "customer_name": "Sam"})

        asyncio.run(place())
        self.http.post("/robot/claim")
        self.http.post("/robot/orders/1/done", json={"box_id": "B1"})
        evs = self.http.get("/api/events").json()
        texts = [e["text"] for e in evs]
        self.assertIn("Agent called place_order", texts)
        self.assertEqual([e["data"]["ok"] for e in evs if e["kind"] == "tool"], [True, False])
        self.assertIn("Robot started packing order 1 for Sam", texts)
        self.assertIn("Order 1 packed", texts)
        self.assertEqual(self.http.get("/api/events", params={"after": evs[-1]["seq"]}).json(), [])

    def test_robot_state(self):
        self.assertEqual(self.http.get("/api/state").json()["robot"]["state"], "offline")
        self.http.post("/robot/claim")  # check-in with nothing to do
        self.assertEqual(self.http.get("/api/state").json()["robot"]["state"], "idle")
        ShopTools(self.store, self.catalog).place_order(["A"], "Sam")
        self.http.post("/robot/claim")
        s = self.http.get("/api/state").json()
        self.assertEqual((s["robot"]["state"], s["robot"]["order"]["id"], s["counts"]), ("packing", 1, {"packing": 1}))

    def test_dashboard_page_served(self):
        r = self.http.get("/dashboard", follow_redirects=False)
        self.assertEqual((r.status_code, r.headers["location"]), (307, "/dashboard/"))
        r = self.http.get("/dashboard/")
        self.assertEqual(r.status_code, 200)
        self.assertIn("/dashboard/assets/", r.text)  # the built React app (packing/shop/web)


class ShelfTests(unittest.TestCase):
    """Real stock from the cell inventory database."""

    def test_restock_consume_and_sim_packer(self):
        import tempfile
        from pathlib import Path
        from types import SimpleNamespace as NS

        from packing.shop.shelf import Shelf, ShelfSimPacker

        tmp = tempfile.TemporaryDirectory()
        cfg = NS(inventory_db=Path(tmp.name) / "inv.db", stock_slots={"a1": "A", "a2": "A", "b1": "B"},
                 box_supply=["in1"])
        shelf = Shelf(cfg)
        self.assertEqual(shelf.available(), {})
        self.assertEqual(shelf.restock(), (3, 1))
        self.assertEqual(shelf.available(), {"A": 2, "B": 1})
        store = OrderStore(":memory:")
        catalog = Catalog({"A": "widget", "B": "gadget"}, 2, available=shelf.available)
        http = TestClient(create_server(store, catalog, shelf=shelf))
        ShopTools(store, catalog).place_order(["A", "B"], "Sam")
        http.post("/api/orders/1/action")
        self.assertEqual(run("http://t", ShelfSimPacker(shelf, 0), once=True, http=http), 0)
        self.assertEqual(shelf.available(), {"A": 1})
        stock = {p["sku"]: p for p in http.get("/api/state").json()["stock"]}
        self.assertEqual((stock["A"]["on_shelf"], stock["B"]["on_shelf"]), (1, 0))
        self.assertEqual(http.post("/api/shelf/restock").json(), {"parts_added": 2, "boxes_added": 0})  # the A and B slots just emptied
        tmp.cleanup()
