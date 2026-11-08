"""Remote-agent FastMCP server (packing/shop/mcp_server.py, mounted in the shop server at /agent/mcp): tools in
memory, then the whole shop server over real HTTP (bearer token, Host check for tunnels, /mcp untouched)."""

import asyncio
import socket
import tempfile
import threading
import time
import unittest
from pathlib import Path

import httpx
import uvicorn
from fastmcp import Client
from fastmcp.exceptions import ToolError

from packing.service.orders import OrderStore
from packing.service.vapi import Catalog
from packing.shop.mcp_server import DISABLED, create_mcp
from packing.shop.server import create_server


def shop(tmp):
    store = OrderStore(Path(tmp) / "orders.db")
    catalog = Catalog({"aa": "AA battery", "glue": "glue stick"}, max_items=2, available=lambda: {"aa": 3, "glue": 0})
    return store, catalog


def run(coro):
    return asyncio.run(coro)


class ToolTests(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.store, self.catalog = shop(self._tmp.name)
        self.mcp = create_mcp(self.store, self.catalog)

    def tearDown(self):
        self._tmp.cleanup()

    def call(self, name, args=None):
        async def go():
            async with Client(self.mcp) as c:
                return await c.call_tool(name, args or {})
        return run(go())

    def test_lists_exactly_the_four_tools(self):
        async def go():
            async with Client(self.mcp) as c:
                return sorted(t.name for t in await c.list_tools())
        self.assertEqual(run(go()), ["cancel_order", "check_order", "create_order", "get_inventory"])

    def test_inventory_then_order_reserves_stock(self):
        inv = self.call("get_inventory").data
        self.assertEqual({p["sku"]: p["available"] for p in inv["products"]}, {"aa": 3, "glue": 0})
        self.assertEqual(inv["max_items_per_order"], 2)
        out = self.call("create_order", {"customer_name": "Sam", "items": ["AA battery", "aa"]}).data
        self.assertEqual(out["status"], "received")
        order = self.store.get(out["order_id"])
        self.assertEqual((order.items, order.status, order.source), (["aa", "aa"], "queued", "mcp-remote"))
        inv = self.call("get_inventory").data
        self.assertEqual({p["sku"]: p["available"] for p in inv["products"]}["aa"], 1)

    def test_bad_orders_are_tool_errors_and_change_nothing(self):
        with self.assertRaises(ToolError) as e:
            self.call("create_order", {"customer_name": "Sam", "items": ["glue stick"]})
        self.assertIn("not enough stock", str(e.exception))
        with self.assertRaises(ToolError):
            self.call("create_order", {"customer_name": "", "items": ["aa"]})
        self.assertEqual(self.store.list(limit=10), [])

    def test_stubs_change_nothing(self):
        oid = self.call("create_order", {"customer_name": "Sam", "items": ["aa"]}).data["order_id"]
        self.assertEqual(self.call("cancel_order", {"order_id": oid}).data, DISABLED)
        self.assertEqual(self.call("check_order", {"order_id": oid}).data, DISABLED)
        self.assertEqual(self.store.get(oid).status, "queued")


def free_port():
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def start_server(app):
    port = free_port()
    server = uvicorn.Server(uvicorn.Config(app, host="127.0.0.1", port=port, log_level="warning"))
    threading.Thread(target=server.run, daemon=True).start()
    for _ in range(200):
        if server.started:
            break
        time.sleep(0.05)
    return server, f"http://127.0.0.1:{port}"


class HttpTests(unittest.TestCase):
    """The whole shop server over real HTTP: what a cloud agent behind a tunnel sees at /agent/mcp."""

    @classmethod
    def setUpClass(cls):
        cls._tmp = tempfile.TemporaryDirectory()
        store, catalog = shop(cls._tmp.name)
        cls.server, base = start_server(create_server(store, catalog, agent_token="s3cret",
                                                      agent_hosts=["*.trycloudflare.com"]))
        cls.base, cls.url = base, f"{base}/agent/mcp"

    @classmethod
    def tearDownClass(cls):
        cls.server.should_exit = True
        time.sleep(0.2)
        cls._tmp.cleanup()

    INIT = {"jsonrpc": "2.0", "id": 1, "method": "initialize",
            "params": {"protocolVersion": "2025-06-18", "capabilities": {}, "clientInfo": {"name": "t", "version": "1"}}}
    HDR = {"Accept": "application/json, text/event-stream", "Content-Type": "application/json"}

    def test_client_with_token_can_order(self):
        async def go():
            async with Client(self.url, auth="s3cret") as c:
                inv = (await c.call_tool("get_inventory", {})).data
                out = (await c.call_tool("create_order", {"customer_name": "Remote", "items": ["aa"]})).data
                return inv, out
        inv, out = run(go())
        self.assertEqual(inv["products"][0]["sku"], "aa")
        self.assertEqual(out["status"], "received")

    def test_missing_or_wrong_token_is_401(self):
        self.assertEqual(httpx.post(self.url, json=self.INIT, headers=self.HDR).status_code, 401)
        bad = {**self.HDR, "Authorization": "Bearer nope"}
        self.assertEqual(httpx.post(self.url, json=self.INIT, headers=bad).status_code, 401)

    def test_tunnel_host_allowed_other_hosts_refused(self):
        ok = {**self.HDR, "Authorization": "Bearer s3cret", "Host": "abc-def.trycloudflare.com"}
        self.assertEqual(httpx.post(self.url, json=self.INIT, headers=ok).status_code, 200)
        evil = {**ok, "Host": "evil.example.com"}
        self.assertNotEqual(httpx.post(self.url, json=self.INIT, headers=evil).status_code, 200)

    def test_chat_mcp_at_slash_mcp_is_untouched(self):
        async def go():
            async with Client(f"{self.base}/mcp") as c:  # the dashboard chat's tool set, no token needed
                return sorted(t.name for t in await c.list_tools())
        self.assertEqual(run(go()), ["cancel_order", "check_order", "list_products", "place_order"])


class DefaultsTests(unittest.TestCase):
    def test_open_and_any_host_by_default(self):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        server, base = start_server(create_server(*shop(tmp.name)))
        self.addCleanup(setattr, server, "should_exit", True)
        hdr = {**HttpTests.HDR, "Host": "whatever.ngrok-free.app"}
        self.assertEqual(httpx.post(f"{base}/agent/mcp", json=HttpTests.INIT, headers=hdr).status_code, 200)


if __name__ == "__main__":
    unittest.main()
