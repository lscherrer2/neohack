"""Order service tests (no network, no hardware): python -m unittest discover -s packing/tests -t ."""

import json
import unittest

import httpx
from fastapi.testclient import TestClient

from packing.service.app import create_app
from packing.service.orders import OrderStore
from packing.service.vapi import Catalog, VapiNotifier, VapiSettings
from packing.service.worker import SimPacker, Worker


def tool_call(name, args, call_id="tc1", caller="+15551234567"):
    return {"message": {"type": "tool-calls",
                        "call": {"id": "call-1", "customer": {"number": caller}},
                        "toolCallList": [{"id": call_id, "type": "function",
                                          "function": {"name": name, "arguments": args}}]}}


class ServiceTests(unittest.TestCase):
    def setUp(self):
        self.sent = []

        def vapi(request: httpx.Request):
            self.sent.append((request.url.path, request.headers.get("authorization"), json.loads(request.content)))
            return httpx.Response(201, json={"id": "out-call-9", "status": "queued"})

        self.store = OrderStore(":memory:")
        self.stock = {"A": 3, "B": 1}
        self.catalog = Catalog({"A": "widget", "B": "gadget"}, max_items=2, available=lambda: self.stock)
        settings = VapiSettings("key123", "phone-1", "asst-cb", server_secret="s3cret")
        notifier = VapiNotifier(settings, self.catalog,
                                httpx.Client(base_url="https://api.vapi.ai", transport=httpx.MockTransport(vapi)))
        self.worker = Worker(self.store, SimPacker(0), notifier)
        app = create_app(self.store, self.catalog, self.worker, settings, admin_token="adm", start_worker=False)
        self.c = TestClient(app)
        self.h = {"X-Vapi-Secret": "s3cret"}

    def hook(self, body, headers=None):
        r = self.c.post("/vapi/webhook", json=body, headers=self.h if headers is None else headers)
        return r

    def result(self, r):
        self.assertEqual(r.status_code, 200)
        return r.json()["results"][0]

    def test_rejects_bad_secret(self):
        self.assertEqual(self.hook(tool_call("list_products", {}), {"X-Vapi-Secret": "nope"}).status_code, 401)
        self.assertEqual(self.hook(tool_call("list_products", {}), {"Authorization": "Bearer s3cret"}).status_code, 200)

    def test_list_products(self):
        res = self.result(self.hook(tool_call("list_products", {})))
        self.assertEqual(res["toolCallId"], "tc1")
        self.assertIn("widget (3 in stock)", res["result"])

    def test_place_order_uses_caller_number_and_is_idempotent(self):
        args = {"customer_name": "Sam", "items": [{"product": "widget", "quantity": 2}]}
        res = self.result(self.hook(tool_call("place_order", args)))
        self.assertIn("Order number 1", res["result"])
        self.assertIn("+15551234567", res["result"])
        again = self.result(self.hook(tool_call("place_order", args)))  # Vapi retry, same toolCallId
        self.assertIn("Order number 1", again["result"])
        self.assertEqual(len(self.store.list()), 1)
        self.assertEqual(self.store.get(1).items, ["A", "A"])

    def test_stock_and_size_limits_are_spoken_not_errors(self):
        r1 = self.result(self.hook(tool_call("place_order", {"items": [{"product": "B", "quantity": 2}]})))
        self.assertIn("only 1 left", r1["result"])
        r2 = self.result(self.hook(tool_call("place_order", {"items": ["A", "A", "B"]}, "tc2")))
        self.assertIn("up to 2 items", r2["result"])
        r3 = self.result(self.hook(tool_call("place_order", {"items": ["sprocket"]}, "tc3")))
        self.assertIn("don't carry", r3["result"])
        self.assertEqual(self.store.list(), [])

    def test_queued_orders_reserve_stock(self):
        self.result(self.hook(tool_call("place_order", {"items": ["gadget"]}, "tc1")))
        r = self.result(self.hook(tool_call("place_order", {"items": ["gadget"]}, "tc2")))
        self.assertIn("only 0 left", r["result"])

    def test_full_cycle_pack_then_callback(self):
        self.hook(tool_call("place_order", {"customer_name": "Sam", "items": ["widget", "gadget"]}))
        self.assertIn("in line", self.result(self.hook(tool_call("check_order", {}, "tc2")))["result"])
        self.assertTrue(self.worker.step())
        o = self.store.get(1)
        self.assertEqual((o.status, o.box_id, o.callback_call_id), ("called", "SIM-BOX-1", "out-call-9"))
        path, auth, body = self.sent[0]
        self.assertEqual((path, auth), ("/call", "Bearer key123"))
        self.assertEqual(body["customer"]["number"], "+15551234567")
        self.assertEqual(body["assistantId"], "asst-cb")
        self.assertEqual(body["assistantOverrides"]["variableValues"]["items"], "1 widget, 1 gadget")
        self.assertIn("ready", self.result(self.hook(tool_call("check_order", {"order_id": 1}, "tc3")))["result"])

    def test_cancel_only_when_queued(self):
        self.hook(tool_call("place_order", {"items": ["widget"]}))
        self.assertIn("cancelled", self.result(self.hook(tool_call("cancel_order", {"order_id": 1}, "tc2")))["result"])
        self.assertFalse(self.worker.step())  # nothing left to pack

    def test_pack_failure_pauses_worker_and_retry(self):
        class Boom:
            def pack(self, order):
                raise RuntimeError("gripper missed")

        self.worker.packer = Boom()
        self.hook(tool_call("place_order", {"items": ["widget"]}))
        self.worker.step()
        self.assertEqual(self.store.get(1).status, "failed")
        self.assertTrue(self.worker.paused)
        adm = {"Authorization": "Bearer adm"}
        self.assertEqual(self.c.post("/orders/1/retry", headers=adm).json()["status"], "queued")

    def test_admin_api(self):
        self.assertEqual(self.c.get("/orders").status_code, 401)
        adm = {"Authorization": "Bearer adm"}
        r = self.c.post("/orders", json={"phone": "(555) 765-4321", "items": ["A"]}, headers=adm)
        self.assertEqual(r.status_code, 201)
        self.assertEqual(r.json()["phone"], "+15557654321")
        self.assertEqual(r.json()["queue_position"], 1)
        self.assertEqual(len(self.c.get("/orders", headers=adm).json()), 1)

    def test_other_events_acknowledged(self):
        r = self.hook({"message": {"type": "end-of-call-report", "call": {"id": "x"}}})
        self.assertEqual((r.status_code, r.json()), (200, {}))

    def test_dry_run_without_keys(self):
        n = VapiNotifier(VapiSettings(), self.catalog)
        o, _ = self.store.create("+15550000000", ["A"])
        self.assertEqual(n.notify(o), "dry-run")


if __name__ == "__main__":
    unittest.main()


class ChatEmulatorTests(unittest.TestCase):
    """The terminal agent must send Vapi-shaped tool calls and feed results back (fake model, no API)."""

    def test_tool_round_trip_through_vapi_handler(self):
        from types import SimpleNamespace as NS

        from packing.service.chat import Call, InProcessVapi, anthropic_tools

        store = OrderStore(":memory:")
        catalog = Catalog({"A": "widget"}, max_items=2, available=lambda: {"A": 5})
        replies = [
            NS(stop_reason="tool_use", content=[NS(type="tool_use", id="tu1", name="place_order",
                                                   input={"customer_name": "Sam", "items": [{"product": "widget"}]})]),
            NS(stop_reason="end_turn", content=[NS(type="text", text="Order placed!")]),
        ]
        seen = []

        class FakeMessages:
            def create(self, **kw):
                seen.append([dict(m) if isinstance(m, dict) else m for m in kw["messages"]])
                return replies.pop(0)

        call = Call(NS(messages=FakeMessages()), "sys", anthropic_tools(),
                    InProcessVapi(__import__("packing.service.vapi", fromlist=["ToolHandler"]).ToolHandler(store, catalog)),
                    "+15551234567")
        self.assertEqual(call.turn("one widget please"), "Order placed!")
        self.assertEqual(store.get(1).phone, "+15551234567")  # caller id flowed through the Vapi message
        tool_result = seen[1][-1]["content"][0]
        self.assertEqual((tool_result["tool_use_id"], tool_result["is_error"]), ("tu1", False))
        self.assertIn("Order number 1", tool_result["content"])
