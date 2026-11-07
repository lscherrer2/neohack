"""Shop server: MCP tools for the ordering agent + a tiny job API for the robot agent.

  /mcp                          MCP (Streamable HTTP): list_products, place_order, check_order, cancel_order
  POST /robot/claim             next order to pack (the in-progress one first, so a restarted agent resumes).
                                Manual mode (default): only orders actioned on the dashboard ('ready').
  POST /api/orders/{id}/action  release a received order to the robot (queued -> ready)
  DELETE /api/orders/{id}       remove an order that isn't being packed / packed
  POST /api/shelf/restock       after refilling by hand: mark part slots / box spots as full
  POST /robot/orders/{id}/done      {"box_id": "..."}  -> packed
  POST /robot/orders/{id}/failed    {"error": "..."}   -> failed
  GET  /orders  GET /orders/{id}  GET /health
  GET  /dashboard               live monitoring page (state: /api/state, events: /api/events/stream)
"""

from __future__ import annotations

import asyncio
import json
import time
from contextlib import asynccontextmanager
from pathlib import Path

from fastapi import FastAPI, HTTPException, Request
from fastapi.responses import FileResponse, StreamingResponse
from mcp.server.mcpserver import MCPServer
from mcp.server.transport_security import TransportSecuritySettings
from pydantic import BaseModel, Field

from ..service.orders import OrderError, OrderStore
from ..service.vapi import Catalog
from .chat import Conversation, load_api_key
from .events import EventLog
from .tools import ShopTools

DASHBOARD = Path(__file__).parent / "static" / "dashboard.html"
ROBOT_ONLINE_S = 5.0  # robot agent polls every ~1 s when idle

INSTRUCTIONS = ("Ordering for a robot packing station. Use list_products to see what's available, confirm the "
                "order with the customer, then place_order. Every tool returns a sentence to relay to the customer.")


class OrderItem(BaseModel):
    product: str = Field(description="Product name or SKU from list_products")
    quantity: int = Field(1, ge=1)


def build_mcp(tools: ShopTools, events: EventLog | None = None) -> MCPServer:
    mcp = MCPServer("shop", instructions=INSTRUCTIONS)
    events = events or EventLog()

    def safe(fn, *a, **kw) -> str:
        name = fn.__name__
        args = {k: v for k, v in kw.items() if v is not None}
        try:
            out, ok = fn(*a, **kw), True
        except OrderError as e:  # expected problems are answers, not errors
            out, ok = str(e), False
        except Exception as e:  # noqa: BLE001
            events.record("error", f"{name} crashed: {e}", tool=name, args=args)
            raise
        events.record("tool", f"Agent called {name}", tool=name, args=args, result=out, ok=ok)
        return out

    @mcp.tool(description="List the products that can be ordered and how many are in stock.")
    def list_products() -> str:
        return safe(tools.list_products)

    @mcp.tool(description="Place an order for the robot to pack. Confirm the items and name with the customer first.")
    def place_order(items: list[OrderItem], customer_name: str, phone: str | None = None) -> str:
        return safe(tools.place_order, items=[i.model_dump() for i in items], customer_name=customer_name,
                    phone=phone)

    @mcp.tool(description="Get the status of an order by its order number.")
    def check_order(order_id: int) -> str:
        return safe(tools.check_order, order_id=order_id)

    @mcp.tool(description="Cancel an order that hasn't started packing yet.")
    def cancel_order(order_id: int) -> str:
        return safe(tools.cancel_order, order_id=order_id)

    return mcp


class ChatIn(BaseModel):
    session_id: str = Field(min_length=1, max_length=64)
    message: str = Field(min_length=1, max_length=2000)


class DoneIn(BaseModel):
    box_id: str | None = None


class FailedIn(BaseModel):
    error: str


def create_server(store: OrderStore, catalog: Catalog, events: EventLog | None = None,
                  auto_dispatch: bool = False, shelf=None) -> FastAPI:
    """auto_dispatch=False: the robot only packs orders someone clicked "Action" on."""
    tools = ShopTools(store, catalog)
    events = events or EventLog()
    mcp = build_mcp(tools, events)
    mcp_app = mcp.streamable_http_app(
        streamable_http_path="/mcp", stateless_http=True, json_response=True,
        # Local/tunnel use: Host-header checks would reject non-localhost hostnames.
        transport_security=TransportSecuritySettings(enable_dns_rebinding_protection=False))

    @asynccontextmanager
    async def lifespan(_app):
        events.record("system", "Server started")
        recovered = store.list("packing")  # left in progress by a robot agent; claim hands it back
        if recovered:
            events.record("system", f"Order {recovered[0].id} was mid-pack when the server stopped; "
                                    "the robot agent will resume it", order_id=recovered[0].id)
        async with mcp.session_manager.run():
            yield

    app = FastAPI(title="Shop server", lifespan=lifespan)

    @app.get("/health")
    def health():
        return {"ok": True, "queued": len(store.list("queued")), "packing": [o.id for o in store.list("packing")]}

    @app.get("/orders")
    def list_orders(status: str | None = None, limit: int = 50):
        return [o.to_dict() for o in store.list(status, limit=limit)]

    @app.get("/orders/{order_id}")
    def get_order(order_id: int):
        o = store.get(order_id)
        if not o:
            raise HTTPException(404, "no such order")
        return o.to_dict()

    @app.post("/robot/claim")
    def claim():
        events.robot_seen()
        in_progress = store.list("packing", limit=1)
        if in_progress:
            o = in_progress[0]
            events.record("robot", f"Robot resumed order {o.id}", order_id=o.id, items=o.items)
        else:
            o = store.claim_next(from_status="queued" if auto_dispatch else "ready")
            if o:
                events.record("robot", f"Robot started packing order {o.id} for {o.customer_name}",
                              order_id=o.id, items=o.items)
        return {"order": o.to_dict() if o else None}

    def _packing(order_id: int):
        o = store.get(order_id)
        if not o:
            raise HTTPException(404, "no such order")
        if o.status != "packing":
            raise HTTPException(409, f"order {order_id} is {o.status}, not packing")
        return o

    @app.post("/robot/orders/{order_id}/done")
    def done(order_id: int, body: DoneIn):
        events.robot_seen()
        _packing(order_id)
        o = store.update(order_id, "packed", box_id=body.box_id)
        events.record("order", f"Order {order_id} packed", order_id=order_id, box_id=body.box_id)
        return o.to_dict()

    @app.post("/robot/orders/{order_id}/failed")
    def failed(order_id: int, body: FailedIn):
        events.robot_seen()
        _packing(order_id)
        o = store.update(order_id, "failed", error=body.error)
        events.record("error", f"Order {order_id} failed to pack: {body.error}", order_id=order_id)
        return o.to_dict()

    @app.post("/api/orders/{order_id}/action")
    def action_order(order_id: int):
        try:
            o = store.action(order_id)
        except OrderError as e:
            raise HTTPException(409, str(e)) from e
        events.record("order", f"Order {order_id} actioned; the robot will pack it next", order_id=order_id)
        return o.to_dict()

    @app.delete("/api/orders/{order_id}")
    def delete_order(order_id: int):
        try:
            o = store.delete(order_id)
        except OrderError as e:
            raise HTTPException(409, str(e)) from e
        events.record("order", f"Order {order_id} deleted from the dashboard", order_id=order_id,
                      was=o.status, items=o.items, customer=o.customer_name)
        return {"deleted": order_id}

    @app.post("/api/shelf/restock")
    def restock():
        if shelf is None:
            raise HTTPException(404, "no shelf configured")
        parts, boxes = shelf.restock()
        events.record("order", f"Shelf restocked: {parts} parts and {boxes} boxes added", parts=parts, boxes=boxes)
        return {"parts_added": parts, "boxes_added": boxes}

    # ---------------- dashboard
    def robot_state() -> dict:
        seen = events.robot_last_seen
        ago = None if seen is None else round(time.time() - seen, 1)
        packing = store.list("packing", limit=1)
        failed = store.list("failed", limit=1)
        if packing:
            o = packing[0]
            return {"state": "packing", "order": o.to_dict(), "items": catalog.describe(o.items), "last_seen_s": ago}
        if ago is not None and ago <= ROBOT_ONLINE_S:
            return {"state": "idle", "last_seen_s": ago,
                    "last_failure": failed[0].to_dict() if failed else None}
        return {"state": "offline", "last_seen_s": ago}

    # ---------------- web chat: the same Haiku ordering agent, using the MCP tools in-process
    chat_state: dict = {"claude": None, "tools": None}
    sessions: dict[str, Conversation] = {}

    @app.post("/api/chat")
    async def chat_turn(body: ChatIn):
        import anthropic
        from mcp import Client

        if chat_state["claude"] is None:
            try:
                load_api_key()
            except SystemExit as e:
                return {"error": f"Chat is off: {e}"}
            chat_state["claude"] = anthropic.AsyncAnthropic()
        short = body.session_id[-4:]
        events.record("chat", f"Customer {short}: {body.message}", session=body.session_id)
        try:
            async with Client(mcp) as client:
                if chat_state["tools"] is None:
                    listed = await client.list_tools()
                    chat_state["tools"] = [{"name": t.name, "description": t.description or "",
                                            "input_schema": t.input_schema} for t in listed.tools]
                convo = sessions.get(body.session_id)
                if convo is None:
                    if len(sessions) >= 100:  # keep memory bounded
                        sessions.pop(next(iter(sessions)))
                    convo = sessions[body.session_id] = Conversation(
                        chat_state["claude"], None, chat_state["tools"], verbose=False)
                convo.mcp = client
                reply = await convo.turn(body.message)
        except anthropic.APIError as e:
            events.record("error", f"Chat model error: {e}", session=body.session_id)
            return {"error": f"The ordering agent couldn't answer ({e.__class__.__name__}). Try again."}
        events.record("chat", f"Shop {short}: {reply}", session=body.session_id)
        return {"reply": reply, "tool_calls": convo.last_calls}

    @app.post("/api/chat/{session_id}/reset")
    def chat_reset(session_id: str):
        sessions.pop(session_id, None)
        return {"ok": True}

    @app.get("/dashboard", include_in_schema=False)
    def dashboard():
        return FileResponse(DASHBOARD)

    @app.get("/api/state")
    def state():
        orders = store.list(limit=30)
        counts: dict[str, int] = {}
        for o in store.list(limit=10000):
            counts[o.status] = counts.get(o.status, 0) + 1
        avail, reserved = catalog.available(), store.reserved()
        stock = [{"sku": sku, "name": name, "on_shelf": avail.get(sku, 0), "reserved": reserved.get(sku, 0)}
                 for sku, name in catalog.products.items()]
        return {"robot": robot_state(), "counts": counts, "auto_dispatch": auto_dispatch, "stock": stock,
                "orders": [{**o.to_dict(), "items_text": catalog.describe(o.items)} for o in orders]}

    @app.get("/api/events")
    def list_events(after: int = 0):
        return events.since(after)

    @app.get("/api/events/stream")
    async def stream(request: Request, after: int = 0):
        resume = request.headers.get("last-event-id")  # browser reconnect: continue where it left off
        start = int(resume) if resume and resume.isdigit() else after

        async def gen():
            last, idle = start, 0
            while not await request.is_disconnected():
                new = events.since(last)
                for ev in new:
                    last = ev["seq"]
                    yield f"id: {ev['seq']}\ndata: {json.dumps(ev)}\n\n"
                idle = 0 if new else idle + 1
                if idle >= 30:  # keep-alive comment every ~15 s
                    idle = 0
                    yield ": ping\n\n"
                await asyncio.sleep(0.5)

        return StreamingResponse(gen(), media_type="text/event-stream",
                                 headers={"Cache-Control": "no-cache", "X-Accel-Buffering": "no"})

    app.mount("/", mcp_app)  # after the routes above, so /mcp falls through to the MCP app
    return app
