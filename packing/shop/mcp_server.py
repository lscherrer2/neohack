"""FastMCP server for remote LLM agents (e.g. a cloud agent through a tunnel), part of the shop server:

    python -m packing.shop start [--robot] [--mcp-token SECRET] [--allow-host '*.trycloudflare.com']
    -> http://127.0.0.1:8000/agent/mcp

Streamable HTTP, stateless (no session affinity needed behind a tunnel). Separate from the server's /mcp, which
keeps the full tool set the dashboard chat agent uses. Same data as the dashboard:
stock from the cell inventory (`shelf.Shelf`), orders in packing/data/shop_orders.db. New orders arrive
`queued` ("received" on the dashboard); with manual dispatch (the default) the robot only packs them after a
human presses Action, so this server never moves the arm.

Tools:
  get_inventory  — products with what's available to order (stock minus open orders)
  create_order   — places an order (one box holds `max_items` items); it waits for Action on the dashboard
  check_order    — status + place in line
  cancel_order   — cancels a received/ready order (not once packing has started)

Security: a tunnel makes this public. Pass --mcp-token (or env SHOP_MCP_TOKEN) to require
`Authorization: Bearer <token>`. Host header: any by default (like /mcp; tunnel hostnames change); --allow-host
(repeatable, wildcards like '*.trycloudflare.com') turns on the Host check with localhost + those hosts.
"""

from __future__ import annotations

from typing import Annotated

from pydantic import Field

from ..service.orders import OrderError, OrderStore
from ..service.vapi import Catalog
from .tools import ShopTools

INSTRUCTIONS = (
    "Tools for a small parts shop whose orders are packed by a robot arm. Use get_inventory to see what can be "
    "ordered and how many are available, and create_order to place an order (customer name + item names "
    "exactly as get_inventory lists them, or their sku). check_order reports an order's status; cancel_order "
    "cancels one that hasn't started packing. New orders wait until the shop releases them to the robot."
)

def create_mcp(store: OrderStore, catalog: Catalog, token: str | None = None):
    """FastMCP server with the shop tools. `token`: require this bearer token (None = open)."""
    from fastmcp import FastMCP
    from fastmcp.exceptions import ToolError

    auth = None
    if token:
        from fastmcp.server.auth.providers.jwt import StaticTokenVerifier

        auth = StaticTokenVerifier(tokens={token: {"client_id": "shop-agent", "scopes": []}})
    tools = ShopTools(store, catalog)
    mcp = FastMCP("packing-shop", instructions=INSTRUCTIONS, auth=auth)

    @mcp.tool
    def get_inventory() -> dict:
        """What can be ordered right now: every product with its sku, name and how many are available
        (stock in the parts tray minus items reserved by open orders)."""
        stock = tools.in_stock()
        return {"products": [{"sku": sku, "name": name, "available": stock.get(sku, 0)}
                             for sku, name in catalog.products.items()],
                "max_items_per_order": catalog.max_items,
                "summary": tools.list_products()}

    @mcp.tool
    def create_order(
        customer_name: Annotated[str, Field(description="Name to put the order under")],
        items: Annotated[list[str], Field(description="Product names or skus from get_inventory, one entry per item "
                                                      "(repeat a name to order more than one)")],
        phone: Annotated[str | None, Field(description="Optional phone number for a call-back")] = None,
    ) -> dict:
        """Place an order. It joins the queue as 'received'; the shop releases it to the robot for packing."""
        try:
            order, text = tools.create_order([{"product": i} for i in items], customer_name, phone, source="mcp-remote")
        except OrderError as e:
            raise ToolError(str(e)) from e
        return {"order_id": order.id, "status": "received", "items": catalog.describe(order.items), "message": text}

    @mcp.tool
    def check_order(order_id: Annotated[int, Field(description="Order number from create_order")]) -> dict:
        """Status of an order: received (waiting for the shop), ready, packing, packed, failed or cancelled,
        plus its place in line while it waits."""
        try:
            text = tools.check_order(order_id)
        except OrderError as e:
            raise ToolError(str(e)) from e
        o = store.get(order_id)
        return {"order_id": o.id, "status": "received" if o.status == "queued" else o.status,
                "items": catalog.describe(o.items), "queue_position": store.queue_position(o.id), "message": text}

    @mcp.tool
    def cancel_order(order_id: Annotated[int, Field(description="Order number from create_order")]) -> dict:
        """Cancel an order that hasn't started packing yet (received or ready). Frees its reserved stock."""
        try:
            text = tools.cancel_order(order_id)
        except OrderError as e:
            raise ToolError(str(e)) from e
        return {"order_id": order_id, "status": "cancelled", "message": text}

    return mcp


def http_app(mcp, allow_hosts: list[str] | None = None, any_host: bool = False):
    """ASGI app: streamable HTTP at /mcp, stateless (fine behind tunnels / load balancers)."""
    if any_host:
        return mcp.http_app(path="/mcp", stateless_http=True, host_origin_protection=False)
    hosts = ["127.0.0.1", "127.0.0.1:*", "localhost", "localhost:*", *(allow_hosts or [])]
    return mcp.http_app(path="/mcp", stateless_http=True, host_origin_protection=True, allowed_hosts=hosts)
