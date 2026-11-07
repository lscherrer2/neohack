"""MCP endpoint (Streamable HTTP at /mcp): the same 4 order tools as the webhook, for Vapi's `mcp` tool type.

Vapi lists the tools at the start of each call and opens a session per tool call, so the server runs
stateless with plain JSON responses.

Caller info: MCP requests don't carry Vapi's call object. Configure the Vapi MCP tool with headers
  X-Caller-Number: {{customer.number}}     X-Call-Id: {{call.id}}
and/or have the assistant pass `phone` explicitly (its prompt can include {{customer.number}}).
"""

from __future__ import annotations

from mcp.server.mcpserver import Context, MCPServer
from mcp.server.transport_security import TransportSecuritySettings
from pydantic import BaseModel, Field

from .orders import OrderError
from .vapi import ToolHandler

INSTRUCTIONS = (
    "Phone ordering for a robot packing station. Use list_products to see what's available, confirm the "
    "order and callback number with the caller, then place_order. Every tool returns a sentence to read "
    "to the caller."
)


class OrderItem(BaseModel):
    product: str = Field(description="Product name or SKU from list_products")
    quantity: int = Field(1, ge=1)


def build_mcp(handler: ToolHandler) -> MCPServer:
    server = MCPServer("packing-orders", instructions=INSTRUCTIONS)

    def run(name: str, args: dict, ctx: Context | None) -> str:
        h = {k.lower(): v for k, v in ((ctx.headers if ctx else None) or {}).items()}
        caller = h.get("x-caller-number") or None
        if caller and "{{" in caller:  # template variable Vapi didn't fill in
            caller = None
        try:
            return handler.dispatch(name, args, caller=caller, call_id=h.get("x-call-id"))
        except OrderError as e:
            return str(e)

    @server.tool(description="List the products that can be ordered and how many are in stock.")
    def list_products(ctx: Context) -> str:
        return run("list_products", {}, ctx)

    @server.tool(description="Place an order to be packed. The customer gets a phone call back when it is "
                             "ready. Confirm the items and the callback number with the customer first.")
    def place_order(items: list[OrderItem], ctx: Context, customer_name: str | None = None,
                    phone: str | None = None) -> str:
        """phone: callback number; omit to use the number they called from."""
        return run("place_order", {"items": [i.model_dump() for i in items], "customer_name": customer_name,
                                   "phone": phone}, ctx)

    @server.tool(description="Get the status of an order. Without order_id, returns the caller's latest order.")
    def check_order(ctx: Context, order_id: int | None = None) -> str:
        return run("check_order", {"order_id": order_id}, ctx)

    @server.tool(description="Cancel an order that hasn't started packing yet.")
    def cancel_order(order_id: int, ctx: Context) -> str:
        return run("cancel_order", {"order_id": order_id}, ctx)

    return server


def mcp_asgi_app(server: MCPServer):
    # Host-header (DNS rebinding) protection would reject the tunnel's public hostname; this server is
    # meant to be reached through a tunnel, and auth is the shared secret checked in app.py.
    return server.streamable_http_app(
        streamable_http_path="/mcp", stateless_http=True, json_response=True,
        transport_security=TransportSecuritySettings(enable_dns_rebinding_protection=False))
