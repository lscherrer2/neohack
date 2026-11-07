"""Shop server: MCP tools for the ordering agent + a tiny job API for the robot agent.

  /mcp                          MCP (Streamable HTTP): list_products, place_order, check_order, cancel_order
  POST /robot/claim             next order to pack (the in-progress one first, so a restarted agent resumes).
                                Manual mode (default): only orders actioned on the dashboard ('ready').
  POST /api/orders/{id}/action  release a received order to the robot (queued -> ready)
  DELETE /api/orders/{id}       remove an order that isn't being packed / packed
  POST /api/shelf/restock       after refilling by hand: mark part slots / box spots as full
  GET  /api/tray                parts tray: pockets (DXF, mm), which are full, calibration (tray -> robot)
  POST /api/tray/pockets/{id}   {"full": bool}  a human marks one pocket full / empty
  POST /api/tray/fill           {"full": bool, "sku": optional}  mark every pocket (of one item) full / empty
  POST /robot/orders/{id}/done      {"box_id": "..."}  -> packed
  POST /robot/orders/{id}/failed    {"error": "..."}   -> failed
  GET  /orders  GET /orders/{id}  GET /health
  GET  /dashboard/              React app (packing/shop/web, built into static/app): Orders + Calibration tabs
  GET  /api/arm  /api/arm/stream   live arm state (joints, FK link frames, tip); POST /api/arm/connect|disconnect
  /api/calibration[...]         tray calibration from the dashboard (start, capture, points, save, finish)
  POST /robot/telemetry         joints from a robot agent running in its own process
"""

from __future__ import annotations

import asyncio
import json
import time
from contextlib import asynccontextmanager
from pathlib import Path

from fastapi import FastAPI, HTTPException, Request
from fastapi.responses import FileResponse, RedirectResponse, StreamingResponse
from fastapi.staticfiles import StaticFiles
from mcp.server.mcpserver import MCPServer
from mcp.server.transport_security import TransportSecuritySettings
from pydantic import BaseModel, Field

from ..service.orders import OrderError, OrderStore
from ..service.vapi import Catalog
from .chat import Conversation, load_api_key
from .events import EventLog
from .tools import ShopTools

DASHBOARD = Path(__file__).parent / "static" / "dashboard.html"  # old single-file dashboard (fallback)
APP_DIR = Path(__file__).parent / "static" / "app"  # built React app
ARM_STREAM_S = 0.1
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


class PocketIn(BaseModel):
    full: bool


class FillIn(BaseModel):
    full: bool = True
    sku: str | None = None


class TelemetryIn(BaseModel):
    joints: dict[str, float]
    torque: bool = True
    source: str = "remote"


class ConfirmIn(BaseModel):
    confirm_hold: bool = False


class CaptureIn(BaseModel):
    pocket: str


class DoneIn(BaseModel):
    box_id: str | None = None


class FailedIn(BaseModel):
    error: str


def create_server(store: OrderStore, catalog: Catalog, events: EventLog | None = None,
                  auto_dispatch: bool = False, shelf=None, hub=None) -> FastAPI:
    """auto_dispatch=False: the robot only packs orders someone clicked "Action" on.
    hub: ArmHub (live arm view + calibration tab); None = no arm in this server."""
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
        if hub is not None and hub.mode == "calibrating":
            return {"order": None}  # motors are off for calibration: no packing until it's finished
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

    # ---------------- parts tray
    def _tray():
        tray = getattr(getattr(shelf, "cfg", None), "tray", None)
        if tray is None:
            raise HTTPException(404, "no parts tray configured")
        return tray

    @app.get("/api/tray")
    def tray_state():
        from ..tray.calibration import tilt_deg

        tray = _tray()
        layout, full, cal = tray.layout(), shelf.full_slots(), tray.calibration()
        pockets = []
        for p in layout.pockets:
            d = {"id": p.id, "sku": p.sku, "x": p.x, "y": p.y, "yaw_deg": p.yaw_deg, "shape": p.shape,
                 "full": p.id in full}
            if cal:
                d["robot_m"] = [round(float(v), 5) for v in cal.to_robot(p.x, p.y)]
            pockets.append(d)
        return {
            "outline": layout.outline, "unmatched": layout.unmatched, "protrusion_mm": tray.protrusion_mm,
            "types": [{"sku": t.sku, "name": t.name, "feature": t.feature, "grip_axis": t.grip_axis} for t in tray.types],
            "pockets": pockets,
            "calibration": None if cal is None else {
                "created": cal.created, "rms_mm": cal.rms_mm, "tilt_deg": round(tilt_deg(cal), 1),
                "matrix": cal.matrix(), "reference_points": [p.pocket for p in cal.points]},
        }

    @app.post("/api/tray/pockets/{pocket_id}")
    def set_pocket(pocket_id: str, body: PocketIn):
        _tray()
        try:
            changed = shelf.set_slot(pocket_id, body.full)
        except KeyError as e:
            raise HTTPException(404, f"no pocket {pocket_id}") from e
        if changed:
            events.record("order", f"Pocket {pocket_id} marked {'full' if body.full else 'empty'}",
                          pocket=pocket_id, full=body.full)
        return {"pocket": pocket_id, "full": body.full, "changed": changed}

    @app.post("/api/tray/fill")
    def fill_tray(body: FillIn):
        tray = _tray()
        if body.sku is not None and body.sku not in {t.sku for t in tray.types}:
            raise HTTPException(404, f"no tray item {body.sku}")
        n = shelf.set_all(body.full, body.sku, only=set(tray.layout().slots()))
        what = catalog.products.get(body.sku, body.sku) if body.sku else "all"
        if n:
            events.record("order", f"Tray: {n} {what} pockets marked {'full' if body.full else 'empty'}",
                          sku=body.sku, full=body.full, changed=n)
        return {"changed": n}

    # ---------------- arm (live 3D view) + tray calibration
    from ..tray.calibration import CalibrationError
    from .arm_hub import ArmBusy

    cal_state: dict = {"session": None}

    def _hub():
        if hub is None:
            raise HTTPException(503, "no arm in this server")
        return hub

    def _session():
        if cal_state["session"] is None:
            from ..tray.session import CalibrationSession

            cal_state["session"] = CalibrationSession(_tray(), pan_xy=hub.pan_xy if hub else None)
        return cal_state["session"]

    def _arm_state() -> dict:
        if hub is None:
            return {"connected": False, "source": None, "torque": None, "mode": "idle", "stale_s": None,
                    "error": "no arm in this server", "joints": None, "frames": None, "tip": None}
        return hub.state()

    @app.post("/robot/telemetry")
    def telemetry(body: TelemetryIn):
        if hub is not None:
            hub.external(body.joints, body.torque, body.source)
        return {"ok": True}

    @app.get("/api/arm")
    def arm_state():
        return _arm_state()

    @app.get("/api/arm/stream")
    async def arm_stream(request: Request):
        async def gen():
            while not await request.is_disconnected():
                yield f"data: {json.dumps(_arm_state())}\n\n"
                await asyncio.sleep(ARM_STREAM_S)

        return StreamingResponse(gen(), media_type="text/event-stream",
                                 headers={"Cache-Control": "no-cache", "X-Accel-Buffering": "no"})

    @app.post("/api/arm/connect")
    def arm_connect():
        try:
            _hub().connect()
        except ArmBusy as e:
            raise HTTPException(409, str(e)) from e
        events.record("robot", f"Arm connected ({hub.source})")
        return _arm_state()

    @app.post("/api/arm/disconnect")
    def arm_disconnect():
        try:
            _hub().disconnect()
        except ArmBusy as e:
            raise HTTPException(409, str(e)) from e
        events.record("robot", "Arm disconnected")
        return _arm_state()

    @app.get("/api/calibration")
    def calibration_state():
        return _session().to_dict()

    @app.post("/api/calibration/start")
    def calibration_start(body: ConfirmIn):
        if not body.confirm_hold:
            raise HTTPException(400, "confirm_hold must be true: someone has to hold the arm before the motors turn off")
        h, sess = _hub(), _session()
        if h.mode == "packing":
            raise HTTPException(409, "the arm is packing an order; wait for it to finish")
        try:
            h.connect()
            h.set_torque(False)
        except ArmBusy as e:
            raise HTTPException(409, str(e)) from e
        h.mode, sess.active = "calibrating", True
        events.record("robot", "Tray calibration started: motors OFF (arm held by hand)")
        return sess.to_dict()

    @app.post("/api/calibration/capture")
    def calibration_capture(body: CaptureIn):
        h, sess = _hub(), _session()
        if not sess.active or h.mode != "calibrating":
            raise HTTPException(409, "start calibration first (motors off)")
        try:
            sess.layout.pocket(body.pocket)
        except KeyError as e:
            raise HTTPException(404, f"no pocket {body.pocket}") from e
        try:
            sess.capture(body.pocket, h.read_tip())
        except ArmBusy as e:
            raise HTTPException(409, str(e)) from e
        return sess.to_dict()

    @app.delete("/api/calibration/points/{pocket_id}")
    def calibration_delete(pocket_id: str):
        sess = _session()
        sess.remove(pocket_id)
        return sess.to_dict()

    @app.post("/api/calibration/save")
    def calibration_save():
        sess = _session()
        try:
            cal = sess.save()
        except CalibrationError as e:
            raise HTTPException(409, str(e)) from e
        events.record("robot", f"Tray calibration saved ({cal.rms_mm:.1f} mm RMS, {len(cal.points)} points)",
                      rms_mm=cal.rms_mm)
        return sess.to_dict()

    @app.post("/api/calibration/finish")
    def calibration_finish(body: ConfirmIn):
        if not body.confirm_hold:
            raise HTTPException(400, "confirm_hold must be true: hold the arm, the motors turn back on where it is")
        h, sess = _hub(), _session()
        if h.mode == "calibrating":
            try:
                h.set_torque(True)
            except ArmBusy as e:
                raise HTTPException(409, str(e)) from e
            h.mode = "idle"
        sess.clear()
        events.record("robot", "Tray calibration finished: motors ON, holding position")
        return sess.to_dict()

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

    if (APP_DIR / "index.html").exists():
        @app.get("/dashboard", include_in_schema=False)
        def dashboard_redirect():
            return RedirectResponse("/dashboard/")

        app.mount("/dashboard", StaticFiles(directory=APP_DIR, html=True), name="dashboard")
    else:  # React app not built yet
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
