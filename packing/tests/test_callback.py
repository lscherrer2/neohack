"""Customer callbacks via Vapi (shop/callback.py): fake HTTP transport, no network."""

import json
import os
import tempfile
import unittest
from pathlib import Path
from unittest import mock

import httpx
from fastapi.testclient import TestClient

from packing.service.orders import OrderStore
from packing.service.vapi import VAPI_API, Catalog
from packing.shop.callback import Callbacks, load_env
from packing.shop.events import EventLog
from packing.shop.server import create_server


class CallbackTests(unittest.TestCase):
    def setUp(self):
        self.store = OrderStore(":memory:")
        self.catalog = Catalog({"A": "AA battery", "B": "breadboard"}, max_items=5, available=lambda: {"A": 5, "B": 5})
        self.events = EventLog()
        self.sent, self.reply = [], (201, {"id": "call-123"})

        def handler(req: httpx.Request):
            self.sent.append((req, json.loads(req.content)))
            return httpx.Response(self.reply[0], json=self.reply[1])

        client = httpx.Client(base_url=VAPI_API, transport=httpx.MockTransport(handler))
        self.cb = Callbacks(self.store, self.catalog, self.events, "key", "num-1", "asst-1", client, background=False)
        self.http = TestClient(create_server(self.store, self.catalog, self.events, auto_dispatch=True,
                                             callbacks=self.cb))

    def order(self, phone="+15551234567", name="Sam"):
        o, _ = self.store.create(phone, ["A", "A", "B"], name)
        self.http.post("/robot/claim")
        return o.id

    def test_packed_calls_customer(self):
        oid = self.order()
        self.http.post(f"/robot/orders/{oid}/done", json={"box_id": "B1"})
        self.assertEqual(len(self.sent), 1)
        req, body = self.sent[0]
        self.assertEqual(req.url.path, "/call")
        self.assertEqual(req.headers["authorization"], "Bearer key")
        self.assertEqual(body["assistantId"], "asst-1")
        self.assertEqual(body["phoneNumberId"], "num-1")
        self.assertEqual(body["customer"], {"number": "+15551234567", "name": "Sam"})
        ov = body["assistantOverrides"]
        self.assertEqual(ov["firstMessage"],
                         "Hi Sam, this is the parts shop. Your order with 2 AA batteries, 1 breadboard is ready for pickup.")
        self.assertEqual(ov["variableValues"]["status"], "ready")
        o = self.store.get(oid)
        self.assertEqual((o.callback_call_id, o.callback_status), ("call-123", "called"))

    def test_failed_calls_too(self):
        oid = self.order(name=None)
        self.http.post(f"/robot/orders/{oid}/failed", json={"error": "missed grip"})
        ov = self.sent[0][1]["assistantOverrides"]
        self.assertEqual(ov["variableValues"]["status"], "failed")
        self.assertTrue(ov["firstMessage"].startswith("Hi there, this is the parts shop. Sorry, we couldn't pack"))

    def test_requeue_does_not_call(self):
        oid = self.order()
        self.http.post(f"/robot/orders/{oid}/requeue", json={"error": "no box"})
        self.assertEqual(self.sent, [])

    def test_no_phone_skips(self):
        oid = self.order(phone="")
        self.http.post(f"/robot/orders/{oid}/done", json={"box_id": "B1"})
        self.assertEqual(self.sent, [])
        self.assertEqual(self.store.get(oid).callback_status, "skipped: no phone number")

    def test_vapi_error_recorded_no_retry(self):
        self.reply = (400, {"message": "bad number"})
        oid = self.order()
        r = self.http.post(f"/robot/orders/{oid}/done", json={"box_id": "B1"})
        self.assertEqual(r.status_code, 200)  # packing report still succeeds
        self.assertEqual(len(self.sent), 1)
        o = self.store.get(oid)
        self.assertEqual(o.status, "packed")
        self.assertTrue(o.callback_status.startswith("failed: Vapi 400"))

    def test_from_env(self):
        with tempfile.TemporaryDirectory() as d, mock.patch.dict(os.environ, {}, clear=False):
            for k in ("VAPI_API_KEY", "VAPI_PHONE_NUMBER_ID", "VAPI_CALLBACK_ASSISTANT_ID"):
                os.environ.pop(k, None)
            f = Path(d) / ".env.local"
            f.write_text("ANTHROPIC_API_KEY=x\nVAPI_API_KEY=k\nexport VAPI_PHONE_NUMBER_ID='n'\n")
            load_env(f)
            self.assertEqual((os.getenv("VAPI_API_KEY"), os.getenv("VAPI_PHONE_NUMBER_ID")), ("k", "n"))
            with mock.patch("packing.shop.callback.load_env"):
                self.assertIsNone(Callbacks.from_env(self.store, self.catalog, self.events))  # assistant id missing

    def test_old_db_gets_column(self):
        with tempfile.TemporaryDirectory() as d:
            import sqlite3

            p = Path(d) / "o.db"
            db = sqlite3.connect(p)
            db.execute("CREATE TABLE orders (id INTEGER PRIMARY KEY AUTOINCREMENT, customer_name TEXT, phone TEXT"
                       " NOT NULL, items TEXT NOT NULL, status TEXT NOT NULL, source TEXT NOT NULL, vapi_call_id TEXT,"
                       " tool_call_id TEXT UNIQUE, box_id TEXT, callback_call_id TEXT, error TEXT,"
                       " created_at TEXT NOT NULL, updated_at TEXT NOT NULL)")
            db.commit()
            db.close()
            s = OrderStore(p)
            o, _ = s.create("+15551234567", ["A"])
            self.assertIsNone(o.callback_status)


if __name__ == "__main__":
    unittest.main()
