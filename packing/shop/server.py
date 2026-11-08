"""Shop server: MCP tools for the ordering agent + a tiny job API for the robot agent.

  /mcp                          MCP (Streamable HTTP): list_products, place_order, check_order, cancel_order
  POST /robot/claim             next order to pack (the in-progress one first, so a restarted agent resumes).
                                Manual mode (default): only orders actioned on the dashboard ('ready').
  POST /api/orders/{id}/action  release a received order to the robot (queued -> ready)
  DELETE /api/orders/{id}       remove an order that isn't being packed / packed
  DELETE /api/orders            remove every such order (keeps packing / packed)
  POST /api/shelf/restock       after refilling by hand: mark every empty tray pocket full
  GET  /api/tray                parts tray: pockets (DXF, mm), which are full, calibration (tray -> robot)
  POST /api/tray/pockets/{id}   {"full": bool}  a human marks one pocket full / empty
  POST /api/tray/fill           {"full": bool, "sku": optional}  mark every pocket (of one item) full / empty
  POST /robot/orders/{id}/done      {"box_id": "..."}  -> packed
  POST /robot/orders/{id}/requeue   {"error": "..."}   refused before moving -> back to queued
  POST /robot/orders/{id}/failed    {"error": "..."}   -> failed
  GET  /orders  GET /orders/{id}  GET /health
  GET  /dashboard/              React app (packing/shop/web, built into static/app): Orders + Calibration tabs
  GET  /api/arm  /api/arm/stream   live arm state (joints, FK link frames, tip); POST /api/arm/connect|disconnect
  GET  /api/arm/preview[/{name}]  generic poses drawn without an arm (dashboard preview toggle)
  POST /api/arm/goto            {"point", "confirm_move", "standoff_mm"}: plan (dry run) or move to a tray point,
                                stand-off out along the tray normal; GET = last move; POST /api/arm/stop halts it
  POST /api/arm/release         {"confirm_hold": true} motors off again during calibration (after a go-to)
  POST /api/arm/torque          {"on": bool, "confirm_hold": bool} lock / release the motors while idle (Controls)
  POST /api/arm/pick            {"pocket", "confirm_move"} test pick: dry run, then run (same runner/Stop as go-to)
  GET  /api/boxes               calibrated packing boxes (+ pose at their location, pack spot, geometry)
  POST /api/boxes/capture       motors off, gripper level along a box tab, jaws closed on it: add that box
  DELETE /api/boxes/{id}        remove a box record (a packed box at output: "Picked up")
  POST /api/boxes/{id}/reset    testing: box put back by hand -> empty at its calibrated supply spot again
  POST /api/arm/box-move        {"box_id", "to": pack|output, "confirm_move"} move a box by its tab (forward only)
  POST /api/orders/{id}/pack-plan  dry run of packing that order (box in, picks + drops, box out); nothing moves
  GET  /api/desk                desk plane: touched points, live fit, saved plane
  POST /api/desk/capture        motors off, jaw tip on the desk: add a point;  DELETE /api/desk/points[/{i}]
  POST /api/desk/save           fit (3+ points, spread out) + save;  DELETE /api/desk: remove the calibration
  /api/calibration[...]         tray calibration from the dashboard (start, mode corners|pockets, capture, points,
                                save, finish)
"""

from __future__ import annotations

import asyncio
import json
import time
from contextlib import asynccontextmanager
from pathlib import Path

import numpy as np
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

    @mcp.tool(description="Place an order for the robot to pack: one entry per product with its quantity (any mix, up to the per-order limit list_products gives). Confirm the items and name with the customer first.")
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


class ReachCompIn(BaseModel):
    enabled: bool | None = None
    lift_mm: float | None = Field(default=None, ge=0, le=40)  # mm higher at reach_comp.at_reach of max reach


class ChatIn(BaseModel):
    session_id: str = Field(min_length=1, max_length=64)
    message: str = Field(min_length=1, max_length=2000)


class PocketIn(BaseModel):
    full: bool


class FillIn(BaseModel):
    full: bool = True
    sku: str | None = None


class AutoIn(BaseModel):
    enabled: bool  # dashboard "Auto pack" toggle


class ConfirmIn(BaseModel):
    confirm_hold: bool = False


class CaptureIn(BaseModel):
    pocket: str  # a pocket id or a plate corner id (corner_bl / corner_tl / corner_br / corner_tr)


class GotoIn(BaseModel):
    point: str  # pocket id or plate corner id
    confirm_move: bool = False  # False = plan only (dry run), True = move
    standoff_mm: float = Field(12.7, ge=5.0, le=100.0)  # distance out along the tray normal


class PickIn(BaseModel):
    pocket: str  # tray pocket id, e.g. aa_1
    confirm_move: bool = False  # False = plan only (dry run), True = move


class TorqueIn(BaseModel):
    on: bool  # True = lock (motors on, hold where it is), False = release (limp)
    confirm_hold: bool = False  # required to release: the arm drops


class BoxMoveIn(BaseModel):
    box_id: int
    to: str  # supply | pack | output
    confirm_move: bool = False  # False = plan only (dry run), True = move


class ModeIn(BaseModel):
    mode: str  # corners | pockets


class DoneIn(BaseModel):
    box_id: str | None = None


class FailedIn(BaseModel):
    error: str


def create_server(store: OrderStore, catalog: Catalog, events: EventLog | None = None,
                  auto_dispatch: bool = False, shelf=None, hub=None, agent_token: str | None = None,
                  agent_hosts: list[str] | None = None, box_packing: bool = False) -> FastAPI:
    """auto_dispatch: initial state of the dashboard's "Auto pack" toggle (PUT /api/auto-pack). Off: the robot only
    packs orders someone clicked "Action" on. On: also the NEWEST received order whose items all have a reserved unit
    (and, with box_packing, once an empty box waits at supply and the pack spot is free).
    box_packing: the robot packs into boxes (TrayPacker, `start --robot`).
    hub: ArmHub (live arm view + calibration tab); None = no arm in this server.
    agent_token / agent_hosts: the remote-agent FastMCP server at /agent/mcp (mcp_server.py) — bearer token
    (None = open) and allowed Host headers (None = any, like /mcp; tunnel hostnames change)."""
    from .mcp_server import create_mcp, http_app as agent_http_app

    tools = ShopTools(store, catalog)
    events = events or EventLog()
    agent_app = agent_http_app(create_mcp(store, catalog, agent_token), agent_hosts, any_host=agent_hosts is None)
    mcp = build_mcp(tools, events)
    mcp_app = mcp.streamable_http_app(
        streamable_http_path="/mcp", stateless_http=True, json_response=True,
        # Local/tunnel use: Host-header checks would reject non-localhost hostnames.
        transport_security=TransportSecuritySettings(enable_dns_rebinding_protection=False))

    @asynccontextmanager
    async def lifespan(_app):
        events.record("system", "Server started")
        if shelf is not None:  # orders made before per-unit reservations existed get their units now
            if changed := store.fill_short(shelf.stock()):
                events.record("system", f"Reserved stock units for orders {', '.join(map(str, changed))}")
        recovered = store.list("packing")  # left in progress by a robot agent; claim hands it back
        if recovered:
            events.record("system", f"Order {recovered[0].id} was mid-pack when the server stopped; "
                                    "the robot agent will resume it", order_id=recovered[0].id)
        async with mcp.session_manager.run(), agent_app.lifespan(agent_app):
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
            o = store.claim_next(from_status="ready")  # actioned by hand: always first
            if o is None and auto["on"]:
                cand, _ = _auto_candidate()
                o = store.claim(cand.id) if cand else None
                if o:
                    events.record("robot", f"Auto pack: order {o.id} (the newest one with all its stock)",
                                  order_id=o.id)
            if o:
                events.record("robot", f"Robot started packing order {o.id} for {o.customer_name}",
                              order_id=o.id, items=o.items)
        return {"order": o.to_dict() if o else None}

    # ---------------- auto pack toggle (dashboard Orders tab)
    auto = {"on": bool(auto_dispatch)}

    def _auto_candidate():
        """(order, None): the newest received order with a reserved unit for every item, if the boxes are ready.
        (None, why) otherwise (shown on the dashboard while auto pack waits)."""
        queued = store.list("queued", limit=1000)  # newest first
        if not queued:
            return None, None
        if box_packing:
            listed = boxes.list()
            if at_pack := next((b for b in listed if b.location == "pack"), None):
                return None, f"box {at_pack.id} is still at the pack spot"
            if not any(b.state == "available" and b.location == "supply" for b in listed):
                return None, "no empty box at the supply spot (add one on the Calibration tab)"
        # enough goods: every item has its own reserved unit (no shelf = no stock tracking: any order)
        # refused before (error set on requeue) = needs a person: only Action sends it again
        fresh = [o for o in queued if not o.error]
        if not fresh:
            return None, "the received orders were refused before; send them with Action"
        o = next((o for o in fresh if shelf is None or (o.items and o.slots and None not in o.slots)), None)
        if o is None:
            return None, "not enough stock in the tray for any received order"
        return o, None

    def _auto_state() -> dict:
        cand, why = _auto_candidate() if auto["on"] else (None, None)
        return {"enabled": auto["on"], "next_order": cand.id if cand else None, "waiting": why}

    @app.get("/api/auto-pack")
    def auto_pack_state():
        return _auto_state()

    @app.put("/api/auto-pack")
    def auto_pack_set(body: AutoIn):
        if body.enabled != auto["on"]:
            auto["on"] = body.enabled
            events.record("order", "Auto pack on: the robot packs the newest order with all its stock once a box is "
                                   "ready" if body.enabled else "Auto pack off: only orders sent with Action are packed")
        return _auto_state()

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
        if auto["on"]:  # check the arm first: from now on only orders sent with Action
            auto["on"] = False
            events.record("order", "Auto pack turned off after the failure; check the arm, then send the next order "
                                   "with Action or turn auto pack back on", order_id=order_id)
        return o.to_dict()

    @app.post("/robot/orders/{order_id}/requeue")
    def requeue(order_id: int, body: FailedIn):
        """The packer refused before the arm moved (no box, pack spot taken, can't plan): back to received."""
        events.robot_seen()
        _packing(order_id)
        o = store.update(order_id, "queued", error=body.error)
        events.record("error", f"Order {order_id} not started: {body.error}. It's back to received — fix that, "
                               f"then Action it again", order_id=order_id)
        return o.to_dict()

    @app.post("/api/orders/{order_id}/action")
    def action_order(order_id: int):
        try:
            o = store.action(order_id)
        except OrderError as e:
            raise HTTPException(409, str(e)) from e
        events.record("order", f"Order {order_id} actioned; the robot will pack it next", order_id=order_id)
        return o.to_dict()

    @app.delete("/api/orders")
    def delete_all_orders():
        gone = store.delete_all()
        if gone:
            events.record("order", f"{len(gone)} orders deleted from the dashboard",
                          order_ids=[o.id for o in gone])
        return {"deleted": [o.id for o in gone]}

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
        parts = shelf.restock()
        events.record("order", f"Shelf restocked: {parts} parts added", parts=parts)
        _refill_reservations()
        return {"parts_added": parts}

    # ---------------- parts tray
    def _tray():
        tray = getattr(getattr(shelf, "cfg", None), "tray", None)
        if tray is None:
            raise HTTPException(404, "no parts tray configured")
        return tray

    @app.get("/api/tray")
    def tray_state():
        from ..tray.calibration import tilt_deg
        from ..tray.session import corner_label

        tray = _tray()
        layout, full, cal = tray.layout(), shelf.full_slots(), tray.calibration()
        reserved, locked = store.reserved_slots(), store.locked_slots()
        pockets = []
        for p in layout.pockets:
            d = {"id": p.id, "sku": p.sku, "x": p.x, "y": p.y, "yaw_deg": p.yaw_deg, "shape": p.shape,
                 "full": p.id in full, "reserved_by": reserved.get(p.id), "locked": p.id in locked}
            if cal:
                d["robot_m"] = [round(float(v), 5) for v in cal.to_robot(p.x, p.y)]
            pockets.append(d)
        return {
            "outline": layout.outline, "unmatched": layout.unmatched, "protrusion_mm": tray.protrusion_mm,
            "types": [{"sku": t.sku, "name": t.name, "feature": t.feature, "grip_axis": t.grip_axis} for t in tray.types],
            "pockets": pockets,
            "corners": [{"id": cid, "label": corner_label(layout, cid), "x": x, "y": y}
                        for cid, (x, y) in layout.corners().items()],
            "calibration": None if cal is None else {
                "created": cal.created, "rms_mm": cal.rms_mm, "tilt_deg": round(tilt_deg(cal), 1),
                "matrix": cal.matrix(), "reference_points": [p.pocket for p in cal.points]},
        }

    def _refill_reservations() -> None:
        """Units became available: give short order items one."""
        for oid in store.fill_short(shelf.stock()):
            o = store.get(oid)
            events.record("order", f"Order {oid} now has all its stock reserved" if all(o.slots)
                          else f"Order {oid} got more of its stock reserved", order_id=oid)

    def _emptied(slots: list[str]) -> None:
        """Units are gone: move their reservations to other free units, or mark the items short."""
        stock = shelf.stock()
        for slot in slots:
            oid = store.release_slot(slot, stock)
            if oid is None:
                continue
            o = store.get(oid)
            if None in o.slots:
                events.record("error", f"Order {oid} is short: {slot} was marked empty and no other unit is free",
                              order_id=oid, pocket=slot)
            else:
                events.record("order", f"Order {oid}: {slot} was marked empty; reserved another unit instead",
                              order_id=oid, pocket=slot)

    @app.post("/api/tray/pockets/{pocket_id}")
    def set_pocket(pocket_id: str, body: PocketIn):
        _tray()
        if pocket_id in store.locked_slots():
            raise HTTPException(409, f"{pocket_id} is locked: its order is being packed")
        try:
            changed = shelf.set_slot(pocket_id, body.full)
        except KeyError as e:
            raise HTTPException(404, f"no pocket {pocket_id}") from e
        if changed:
            events.record("order", f"Pocket {pocket_id} marked {'full' if body.full else 'empty'}",
                          pocket=pocket_id, full=body.full)
            if body.full:
                _refill_reservations()
            else:
                _emptied([pocket_id])
        return {"pocket": pocket_id, "full": body.full, "changed": changed}

    @app.post("/api/tray/fill")
    def fill_tray(body: FillIn):
        tray = _tray()
        if body.sku is not None and body.sku not in {t.sku for t in tray.types}:
            raise HTTPException(404, f"no tray item {body.sku}")
        locked = store.locked_slots()  # units of an order being packed: left alone
        pockets = set(tray.layout().slots()) - locked
        before = shelf.full_slots()
        n = shelf.set_all(body.full, body.sku, only=pockets)
        what = catalog.products.get(body.sku, body.sku) if body.sku else "all"
        if n:
            events.record("order", f"Tray: {n} {what} pockets marked {'full' if body.full else 'empty'}",
                          sku=body.sku, full=body.full, changed=n)
            if body.full:
                _refill_reservations()
            else:
                _emptied(sorted(before - shelf.full_slots()))
        return {"changed": n, "locked": sorted(locked)}

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

    @app.get("/api/arm/preview")
    def arm_preview_list():
        from .arm_hub import PREVIEW_POSES

        return {"poses": [{"name": k, "label": label} for k, (label, _) in PREVIEW_POSES.items()]}

    @app.get("/api/arm/preview/{name}")
    def arm_preview(name: str):
        try:
            return _hub().preview(name)
        except KeyError as e:
            raise HTTPException(404, f"no preview pose {name}") from e
        except ArmBusy as e:
            raise HTTPException(503, str(e)) from e

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

    # ---------------- go-to preview (moves the arm; plan first, then confirm_move)
    goto_state: dict = {"runner": None}

    def _path(h, joints, steps) -> dict:
        """{"path": tool-point path segments} of a planned program (3D view)."""
        from ..motion.program import path_payload

        with h._lock:
            return path_payload(h.model, {j: v for j, v in joints.items() if j != "gripper"}, steps)

    def _plan_failed(message: str, h, joints, e) -> HTTPException:
        """422 whose detail also says how far the plan got and where it failed (3D view)."""
        from ..motion.program import path_payload

        try:
            with h._lock:
                extra = path_payload(h.model, {j: v for j, v in joints.items() if j != "gripper"}, error=e)
        except Exception:  # noqa: BLE001 — never hide the real error behind a drawing problem
            extra = {"path": [], "fail_m": getattr(e, "point", None)}
        return HTTPException(422, {"message": message, **extra})

    def _runner():
        if goto_state["runner"] is None:
            from .goto_runner import GotoRunner

            goto_state["runner"] = GotoRunner(hub, shelf.cfg, events.record)
        return goto_state["runner"]

    def _goto_calibration():
        """Live fit while calibrating (if good), else the saved calibration."""
        sess = _session()
        if sess.active:
            cal, err = sess._fit()
            if cal is not None and err is None:
                return cal, "live fit"
        cal = _tray().calibration()
        if cal is None:
            raise HTTPException(409, "calibrate the tray first (or capture enough points for a good live fit)")
        return cal, "saved calibration"

    from ..boxes.desk import check_desk_clear

    def _desk_plane():
        """The calibrated desk plane (None if not calibrated / no cell config): planned moves stay above it."""
        return shelf.cfg.desk_plane() if shelf is not None else None

    @app.post("/api/arm/goto")
    def arm_goto(body: GotoIn):
        from ..motion.kinematics import PathError
        from ..tray.goto import plan_goto

        h = _hub()
        if not h.connected:
            raise HTTPException(409, "connect the arm first")
        if h.mode in ("packing", "moving") or (goto_state["runner"] and goto_state["runner"].running):
            raise HTTPException(409, f"the arm is {h.mode}; wait for it to finish")
        if h.model is None:
            raise HTTPException(503, "kinematics unavailable (placo not installed?)")
        cal, source = _goto_calibration()
        layout = _tray().layout()
        try:
            xy = layout.point(body.point)
        except KeyError as e:
            raise HTTPException(404, f"no pocket or corner {body.point}") from e
        joints = h.state()["joints"]
        if not joints:
            raise HTTPException(409, "no arm reading yet")
        try:
            with h._lock:
                plan = plan_goto(h.model, body.point, cal.to_robot(*xy), cal.normal, joints, standoff_mm=body.standoff_mm)
                check_desk_clear(h.model, joints, plan.steps, _desk_plane(), "go-to")
        except PathError as e:
            raise _plan_failed(f"can't plan that move: {e}", h, joints, e) from e
        out = {"plan": plan.summary(), "calibration": source, "motors_will_turn_on": not h.torque,
               **_path(h, joints, plan.steps),
               "started": False}
        if body.confirm_move:
            _runner().start(plan)
            out["started"] = True
        return out

    @app.post("/api/arm/pick")
    def arm_pick(body: PickIn):
        """Test pick (packing/tray/pick.py): approach along the normal, grip, pull straight out, lift, hold.
        Dry run unless confirm_move. Doesn't touch inventory (a test; the drop spot isn't defined yet)."""
        from ..motion.kinematics import PathError
        from ..tray.pick import plan_pocket_pick

        h = _hub()
        if not h.connected:
            raise HTTPException(409, "connect the arm first")
        if h.mode in ("packing", "moving") or (goto_state["runner"] and goto_state["runner"].running):
            raise HTTPException(409, f"the arm is {h.mode}; wait for it to finish")
        if h.model is None:
            raise HTTPException(503, "kinematics unavailable (placo not installed?)")
        cal, source = _goto_calibration()
        tray = _tray()
        try:
            tray.layout().pocket(body.pocket)
        except KeyError as e:
            raise HTTPException(404, f"no pocket {body.pocket}") from e
        joints = h.state()["joints"]
        if not joints:
            raise HTTPException(409, "no arm reading yet")
        try:
            with h._lock:
                plan = plan_pocket_pick(h.model, tray, shelf.cfg.gripper, cal, body.pocket, joints)
                # then fold back to sleep holding the part (checked clear of the boxes / tray / desk)
                end = next(st.path[-1] if hasattr(st, "path") else st.pose for st in reversed(plan.steps)
                           if hasattr(st, "path") or isinstance(getattr(st, "pose", None), dict))
                plan.steps += rest_move(h.model, end, shelf.cfg.rest(), _obstacles(h.model.pan_xy), _desk_plane(),
                                        f"pick from {body.pocket}")
                check_desk_clear(h.model, joints, plan.steps, _desk_plane(), f"pick from {body.pocket}")
        except PathError as e:
            raise _plan_failed(f"can't plan that pick: {e}", h, joints, e) from e
        out = {"plan": plan.summary(), "calibration": source, "motors_will_turn_on": not h.torque,
               **_path(h, joints, plan.steps),
               "started": False}
        if body.confirm_move:
            _runner().start(plan)
            out["started"] = True
        return out

    @app.get("/api/arm/goto")
    def arm_goto_state():
        r = goto_state["runner"]
        return {"running": bool(r and r.running), "last": r.last if r else None}

    @app.post("/api/arm/stop")
    def arm_stop():
        r = goto_state["runner"]
        return {"stopping": bool(r and r.stop())}

    @app.post("/api/arm/torque")
    def arm_torque(body: TorqueIn):
        """Controls tab: lock the motors (hold the current pose, with gravity feed-forward) or release them.
        Only while idle — calibration manages the motors itself, and nothing changes mid-move / mid-pack."""
        h = _hub()
        if not h.connected:
            raise HTTPException(409, "connect the arm first")
        if h.mode != "idle" or (goto_state["runner"] and goto_state["runner"].running):
            raise HTTPException(409, f"the arm is {h.mode}; motors can only be switched while idle"
                                     + (" (use the Calibration tab)" if h.mode == "calibrating" else ""))
        if not body.on and not body.confirm_hold:
            raise HTTPException(400, "confirm_hold must be true: hold the arm, it drops when the motors turn off")
        try:
            h.set_torque(body.on)
        except ArmBusy as e:
            raise HTTPException(409, str(e)) from e
        events.record("robot", "Motors locked: holding position" if body.on else "Motors released (limp)")
        return _arm_state()

    @app.post("/api/arm/release")
    def arm_release(body: ConfirmIn):
        if not body.confirm_hold:
            raise HTTPException(400, "confirm_hold must be true: hold the arm, the motors turn off")
        h = _hub()
        if h.mode != "calibrating":
            raise HTTPException(409, "motors can only be released here during calibration")
        try:
            h.set_torque(False)
        except ArmBusy as e:
            raise HTTPException(409, str(e)) from e
        events.record("robot", "Motors OFF again for hand-guided capture")
        return _arm_state()

    # ---------------- packing boxes (packing/boxes/)
    from ..boxes.geometry import (BOX_H, BOX_W, TAB_OUT, TAB_W, CaptureError, GripCfg, blocks_intersect, box_block,
                                  box_from_grasp)
    from ..boxes.desk import corrections_mm
    from ..boxes.geometry import mirror
    from ..boxes.geometry import obstacles as box_obstacles
    from ..boxes.move import box_pose, plan_box_move, rest_move, too_close
    from ..boxes.store import LOCATIONS, BoxError, BoxStore

    boxes = BoxStore(store)

    def _box_cfg():
        if shelf is None:
            raise HTTPException(404, "no cell config")
        return shelf.cfg

    def _pan_xy():
        h = _hub()
        if h.model is None:
            raise HTTPException(503, "kinematics unavailable (placo not installed?)")
        return h.model.pan_xy

    def _box_poses(pan_xy) -> dict[int, np.ndarray]:
        cfg = _box_cfg()
        return {b.id: box_pose(b, b.location, cfg, pan_xy) for b in boxes.list()}

    def _obstacles(pan_xy, skip: int | None = None):
        """3D blocks of the other boxes and of the parts tray (+ its stand)."""
        return box_obstacles(_box_cfg(), _box_poses(pan_xy), skip)

    def _boxes_state() -> dict:
        h = _hub()
        pan = h.model.pan_xy if h.model is not None else None
        cfg = _box_cfg()
        desk = cfg.desk_plane()
        listed = []
        for b in boxes.list():
            d = b.to_dict()
            from ..boxes.geometry import fixed_jaw_side

            d["fixed_jaw_calibrated"] = fixed_jaw_side(b.tool_in_box)  # planning flips it per boxes.fixed_jaw
            d["fixed_jaw_used"] = cfg.boxes.fixed_jaw if cfg.boxes.fixed_jaw in ("top", "bottom") else d["fixed_jaw_calibrated"]
            if pan is not None:
                d["pose"] = box_pose(b, b.location, cfg, pan).round(6).tolist()
                # height change of the pack / output spot vs the calibrated spot, from the desk plane ({} = none)
                d["desk_mm"] = corrections_mm(desk, b.supply[:2, 3], {
                    "pack": np.array(cfg.boxes.pack_spot_mm, float) / 1000, "output": mirror(b.supply, pan)[:2, 3]})
            listed.append(d)
        px, py = cfg.boxes.pack_spot_mm
        return {"boxes": listed, "pack_spot_mm": [px, py],
                "geometry": {"w_m": BOX_W, "h_m": BOX_H, "tab_w_m": TAB_W, "tab_out_m": TAB_OUT}}

    @app.get("/api/boxes")
    def boxes_state():
        return _boxes_state()

    @app.post("/api/boxes/capture")
    def boxes_capture():
        """Motors off, gripper level along the tab, jaws closed up/down on it: add that box."""
        h, cfg = _hub(), _box_cfg()
        if not h.connected:
            raise HTTPException(409, "connect the arm first")
        if h.torque:
            raise HTTPException(409, "release the motors first (hold the arm), then close the jaws on the tab by hand")
        pan = _pan_xy()
        try:
            q = h.read_pose()
            body = {j: v for j, v in q.items() if j != "gripper"}
            with h._lock:
                T_tool = h.model.fk(body)
            g = GripCfg(from_wall_mm=cfg.boxes.grip_from_wall_mm)
            T_box, report = box_from_grasp(T_tool, pan, g, desk=cfg.desk_plane())
        except (ArmBusy, CaptureError) as e:
            raise HTTPException(409 if isinstance(e, ArmBusy) else 422, str(e)) from e
        if T_box[1, 3] < pan[1]:
            raise HTTPException(422, "that box is on the right of the arm — supply boxes go on the LEFT")
        if why := too_close(T_box, pan, cfg):
            raise HTTPException(422, f"that box {why}")
        for ob in _obstacles(pan):  # 3D: a box may sit under the tray's overhang, not inside it
            if blocks_intersect(box_block(T_box), ob, 0.0025):
                raise HTTPException(422, f"that box would overlap {ob.name} — is it already calibrated?")
        from ..boxes.geometry import fixed_jaw_side

        report["fixed_jaw"] = fixed_jaw_side(np.linalg.inv(T_box) @ T_tool)  # as calibrated (planning may flip it)
        b = boxes.add(T_box, np.linalg.inv(T_box) @ T_tool, body, report)
        c = T_box[:3, 3] * 1000
        events.record("robot", f"Box {b.id} added at ({c[0]:.0f}, {c[1]:.0f}) mm", box_id=b.id, **report)
        return _boxes_state()

    @app.delete("/api/boxes/{box_id}")
    def boxes_remove(box_id: int):
        try:
            b = boxes.remove(box_id)
        except BoxError as e:
            raise HTTPException(409 if "in use" in str(e) else 404, str(e)) from e
        events.record("robot", f"Box {box_id} picked up" + (f" (order {b.order_id})" if b.order_id else "")
                      if b.state == "packed" else f"Box {box_id} removed", box_id=box_id)
        return _boxes_state()

    @app.post("/api/boxes/{box_id}/reset")
    def boxes_reset(box_id: int):
        try:
            b = boxes.reset(box_id)
        except BoxError as e:
            raise HTTPException(409 if "in use" in str(e) else 404, str(e)) from e
        events.record("robot", f"Box {box_id} reset to its starting spot (put back by hand)", box_id=b.id)
        return _boxes_state()

    # ---------------- desk plane (packing/boxes/desk.py): box spot heights follow the real desk
    from ..boxes.desk import DeskError, DeskPlane, fit_plane, touch_point
    from ..motion.joint_offsets import OffsetFitError, fit_offsets, load_offsets, save_offsets

    desk_points: list[dict] = []  # touched, not saved yet (in memory): {"tip": xyz m, "joints": body pose}

    def _tips() -> list[list[float]]:
        return [p["tip"] for p in desk_points]

    def _offsets_state() -> dict:
        cfg = _box_cfg()
        f = cfg.joint_offsets_file
        return {"offsets_deg": load_offsets(f), "file": str(f) if f else None}

    def _desk_state() -> dict:
        cfg = _box_cfg()
        saved = cfg.desk_plane()
        fit, err = None, None
        if desk_points:
            try:
                fit = fit_plane(_tips()).to_dict()
            except DeskError as e:
                err = str(e)
        return {"points": _tips(), "fit": fit, "fit_error": err,
                "saved": ({k: v for k, v in saved.to_dict().items() if k != "poses"} | {"has_poses": bool(saved.poses)})
                if saved else None,
                "joint_offsets": _offsets_state()}

    @app.get("/api/desk")
    def desk_state():
        return _desk_state()

    @app.post("/api/desk/capture")
    def desk_capture():
        """Motors off, closed jaw tip touching the desk: add that point (and its joint pose, for the joint fit)."""
        h = _hub()
        if not h.connected:
            raise HTTPException(409, "connect the arm first")
        if h.torque:
            raise HTTPException(409, "release the motors first (hold the arm), then touch the desk with the jaw tip")
        try:
            body, T_tool = h.read_tool()
        except ArmBusy as e:
            raise HTTPException(409, str(e)) from e
        tip = touch_point(T_tool)  # the jaw tips (what touches the desk), not the tool point
        desk_points.append({"tip": [round(float(v), 5) for v in tip], "joints": {j: float(v) for j, v in body.items()}})
        return _desk_state()

    @app.delete("/api/desk/points")
    def desk_clear_points():
        desk_points.clear()
        return _desk_state()

    @app.delete("/api/desk/points/{index}")
    def desk_remove_point(index: int):
        if not 0 <= index < len(desk_points):
            raise HTTPException(404, f"no point {index}")
        desk_points.pop(index)
        return _desk_state()

    @app.post("/api/desk/save")
    def desk_save():
        cfg = _box_cfg()
        try:
            plane = fit_plane(_tips(), [p["joints"] for p in desk_points])
        except DeskError as e:
            raise HTTPException(422, str(e)) from e
        plane.save(cfg.desk_file)
        desk_points.clear()
        events.record("robot", f"Desk calibrated: {len(plane.points)} points, tilt {plane.tilt_deg:.2f}°, "
                               f"RMS {plane.rms_mm:.1f} mm")
        return _desk_state()

    @app.delete("/api/desk")
    def desk_remove():
        cfg = _box_cfg()
        if cfg.desk_file and cfg.desk_file.exists():
            cfg.desk_file.unlink()
            events.record("robot", "Desk calibration removed (box spots use the calibrated box height again)")
        return _desk_state()

    # ---------------- reach compensation (packing/motion/reach_comp.py): aim higher further out
    from ..motion import reach_comp as _rc

    def _reach_state() -> dict:
        cfg, h = _box_cfg(), _hub()
        rc = cfg.reach_comp_settings()
        out = rc.to_dict()
        if h.model is not None:
            with h._lock:
                out |= {"max_reach_mm": round(_rc.max_reach_m(h.model) * 1000), "table": _rc.table(h.model, rc)}
        return out

    @app.get("/api/reach-comp")
    def reach_comp_state():
        return _reach_state()

    @app.put("/api/reach-comp")
    def reach_comp_set(body: ReachCompIn):
        """Turn the reach compensation on/off or change its height (applies from the next move / order)."""
        cfg = _box_cfg()
        rc = cfg.reach_comp_settings()
        if body.enabled is not None:
            rc.enabled = body.enabled
        if body.lift_mm is not None:
            rc.lift_mm = round(body.lift_mm, 1)
        _rc.save(cfg.reach_comp_file, rc)
        events.record("robot", f"Reach compensation {'ON' if rc.enabled else 'off'}: +{rc.lift_mm:.1f} mm at "
                               f"{rc.at_reach:.0%} of max reach")
        return _reach_state()

    # ---------------- joint zero offsets from the desk touches (packing/motion/joint_offsets.py)
    def _touch_poses() -> tuple[list[dict], str]:
        """Joint poses to fit: the touches in memory, else the saved desk calibration's."""
        if desk_points:
            return [p["joints"] for p in desk_points], "touches not saved yet"
        saved = _box_cfg().desk_plane()
        if saved and saved.poses:
            return saved.poses, "the saved desk calibration"
        raise HTTPException(422, "touch the desk first (Capture point) — the joint fit uses those touches"
                                 + (" (the saved desk calibration predates joint poses: re-touch it)" if saved else ""))

    def _offset_fit() -> dict:
        h = _hub()
        if h.model is None:
            raise HTTPException(503, "kinematics unavailable (placo not installed?)")
        poses, source = _touch_poses()
        try:
            with h._lock:
                rep = fit_offsets(h.model, poses)
        except OffsetFitError as e:
            raise HTTPException(422, str(e)) from e
        return rep | {"source": source}

    def _rederive() -> list[str]:
        """After the kinematics changed: recompute everything touch-calibrated from its stored joints.
        Returns what couldn't be (shown to the user)."""
        from ..boxes.geometry import CaptureError as _CaptureError

        h, cfg = _hub(), _box_cfg()
        h.reload_model()
        model = h.model
        notes: list[str] = []
        if model is None:
            return ["kinematics unavailable — nothing re-derived"]
        with h._lock:
            for p in desk_points:
                p["tip"] = [round(float(v), 5) for v in touch_point(model.fk(p["joints"]))]
            saved = cfg.desk_plane()
            if saved and saved.poses:
                tips = [touch_point(model.fk(q)) for q in saved.poses]
                try:
                    fit_plane(tips, saved.poses).save(cfg.desk_file)
                except DeskError as e:
                    notes.append(f"the saved desk calibration no longer fits ({e}) — re-touch the desk")
            elif saved:
                notes.append("the saved desk calibration has no joint poses — re-touch the desk and save it")
            desk = cfg.desk_plane()
            pan = model.pan_xy
            g = GripCfg(from_wall_mm=cfg.boxes.grip_from_wall_mm)
            for b in boxes.list():
                if not b.joints:
                    notes.append(f"box {b.id} has no capture joints — remove and re-add it")
                    continue
                T_tool = model.fk(b.joints)
                try:
                    T_box, _ = box_from_grasp(T_tool, pan, g, desk=desk)
                except _CaptureError as e:
                    notes.append(f"box {b.id}: {e} — remove and re-add it")
                    continue
                boxes.recalibrate(b.id, T_box, np.linalg.inv(T_box) @ T_tool)
        if cfg.tray and cfg.tray.calibration() is not None:
            notes.append("re-calibrate the tray (Calibration tab): its points were measured with the old joint angles")
        return notes

    def _offsets_changeable():
        h = _hub()
        if h.source == "remote":  # the arm (and its model) lives in another process: it wouldn't see the change
            raise HTTPException(409, "the arm is run by another process here — apply joint corrections from "
                                     "`python -m packing.shop start` (the process that owns the arm)")
        if h.mode in ("packing", "moving") or (goto_state["runner"] and goto_state["runner"].running):
            raise HTTPException(409, f"the arm is {h.mode}; wait for it to finish")

    @app.post("/api/joint-offsets/fit")
    def joint_offsets_fit():
        """Dry run: the joint angle corrections that make the desk touches flattest (nothing saved)."""
        return {**_desk_state(), "offset_fit": _offset_fit()}

    @app.post("/api/joint-offsets/apply")
    def joint_offsets_apply():
        """Fit again and save the corrected joint offsets; desk points / plane and boxes are re-derived."""
        _offsets_changeable()
        rep = _offset_fit()
        if not rep["ok"]:
            raise HTTPException(422, "not applied: " + "; ".join(rep["problems"]))
        cfg = _box_cfg()
        save_offsets(cfg.joint_offsets_file, rep["total_deg"],
                     {k: rep[k] for k in ("delta_deg", "std_deg", "rms_before_mm", "rms_after_mm", "source", "note")})
        notes = _rederive()
        events.record("robot", "Joint offsets applied: " + ", ".join(f"{j} {v:+.2f}°" for j, v in rep["delta_deg"].items())
                      + f" (desk touches {rep['rms_before_mm']:.1f} → {rep['rms_after_mm']:.1f} mm RMS)")
        return {**_desk_state(), "offset_fit": rep, "notes": notes}

    @app.delete("/api/joint-offsets")
    def joint_offsets_remove():
        """Back to the plain calibration zero; desk points / plane and boxes are re-derived."""
        _offsets_changeable()
        cfg = _box_cfg()
        notes: list[str] = []
        if cfg.joint_offsets_file and cfg.joint_offsets_file.exists():
            cfg.joint_offsets_file.unlink()
            notes = _rederive()
            events.record("robot", "Joint offsets removed (plain calibration zero again)")
        return {**_desk_state(), "notes": notes}

    @app.post("/api/orders/{order_id}/pack-plan")
    def order_pack_plan(order_id: int):
        """Dry run of packing this order on the arm (box in, picks + drops, box out). Nothing moves, no box is
        assigned. 422 with the reason if it can't be planned."""
        from ..boxes.packer import TrayPacker
        from ..motion.kinematics import PathError

        o = store.get(order_id)
        if not o:
            raise HTTPException(404, "no such order")
        h = _hub()
        if not h.connected:
            raise HTTPException(409, "connect the arm first")
        joints = h.state()["joints"]
        try:
            plan = TrayPacker(h, _box_cfg(), boxes, shelf).plan(o, dry_run=True)
        except PathError as e:
            raise _plan_failed(f"can't pack order {order_id}: {e}", h, joints, e) from e
        except (ValueError, BoxError, RuntimeError) as e:
            raise HTTPException(422, f"can't pack order {order_id}: {e}") from e
        return {"plan": plan.summary() | {"steps": None}, "ok": True, **_path(h, joints, plan.steps)}

    @app.post("/api/arm/box-move")
    def arm_box_move(body: BoxMoveIn):
        """Move a box by its tab (dry run unless confirm_move). Records its new location when done."""
        from ..motion.kinematics import PathError

        h, cfg = _hub(), _box_cfg()
        if body.to not in ("pack", "output"):
            raise HTTPException(400, "to must be pack or output (boxes only move left → right)")
        if not h.connected:
            raise HTTPException(409, "connect the arm first")
        if h.mode in ("packing", "moving") or (goto_state["runner"] and goto_state["runner"].running):
            raise HTTPException(409, f"the arm is {h.mode}; wait for it to finish")
        try:
            b = boxes.get(body.box_id)
        except BoxError as e:
            raise HTTPException(404, str(e)) from e
        if b.location == body.to:
            raise HTTPException(409, f"box {b.id} is already at {body.to}")
        if LOCATIONS.index(body.to) < LOCATIONS.index(b.location):
            raise HTTPException(409, f"box {b.id} is at {b.location}; boxes only move left → right")
        if body.to == "pack" and (other := next((x for x in boxes.list() if x.location == "pack"), None)):
            raise HTTPException(409, f"box {other.id} is still at the pack spot")
        joints = h.state()["joints"]
        if not joints:
            raise HTTPException(409, "no arm reading yet")
        pan = _pan_xy()
        try:
            with h._lock:
                plan = plan_box_move(h.model, b, b.location, body.to, joints, cfg, _obstacles(pan, skip=b.id),
                                     rest=cfg.rest())
        except PathError as e:
            raise _plan_failed(f"can't plan that box move: {e}", h, joints, e) from e

        def moved(box_id=b.id, to=body.to):
            boxes.update(box_id, location=to)
            events.record("robot", f"Box {box_id} is now at {to}", box_id=box_id)
        plan.on_done = moved
        out = {"plan": plan.summary(), "motors_will_turn_on": not h.torque, "started": False,
               **_path(h, joints, plan.steps)}
        if body.confirm_move:
            _runner().start(plan)
            out["started"] = True
        return out

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
        if h.torque:
            raise HTTPException(409, "motors are on (after a go-to) — turn them off to hand-guide, then capture")
        try:
            sess.layout.point(body.pocket)
        except KeyError as e:
            raise HTTPException(404, f"no pocket or corner {body.pocket}") from e
        try:
            sess.capture(body.pocket, h.read_tip())
        except ArmBusy as e:
            raise HTTPException(409, str(e)) from e
        return sess.to_dict()

    @app.post("/api/calibration/mode")
    def calibration_mode(body: ModeIn):
        sess = _session()
        try:
            sess.set_mode(body.mode)
        except ValueError as e:
            raise HTTPException(400, str(e)) from e
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
        return {"robot": robot_state(), "counts": counts, "auto_dispatch": auto["on"], "auto_pack": _auto_state(),
                "stock": stock,
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

    app.mount("/agent", agent_app)  # FastMCP for remote agents: /agent/mcp (inventory + create order)
    app.mount("/", mcp_app)  # after the routes above, so /mcp falls through to the MCP app
    return app
