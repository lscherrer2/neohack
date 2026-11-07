"""HTTP API.

  POST /vapi/webhook          Vapi server URL (tool calls + call events). Always 200.
  GET  /health
  GET  /orders                queue (admin)            ?status=queued
  POST /orders                place an order (admin / testing without a phone call)
  GET  /orders/{id}
  POST /orders/{id}/cancel
  POST /orders/{id}/retry     re-queue a failed order / retry a failed callback
  POST /worker/pause|resume   stop/start taking new orders (current one finishes)
"""

from __future__ import annotations

import hmac
import logging
from contextlib import asynccontextmanager

from fastapi import Depends, FastAPI, Header, HTTPException, Request
from pydantic import BaseModel, Field

from .orders import OrderError, OrderStore, normalize_phone
from .vapi import Catalog, ToolHandler, VapiSettings, parse_items
from .worker import Worker

log = logging.getLogger("packing.app")


class OrderIn(BaseModel):
    phone: str
    items: list[str | dict] = Field(..., description='["A","B"] or [{"product":"A","quantity":2}]')
    customer_name: str | None = None


def _check_secret(expected: str | None, x_vapi_secret: str | None, authorization: str | None) -> None:
    if not expected:
        return  # auth disabled (local development)
    given = x_vapi_secret or (authorization[7:] if authorization and authorization.lower().startswith("bearer ") else None)
    if not given or not hmac.compare_digest(given, expected):
        raise HTTPException(401, "bad secret")


def create_app(store: OrderStore, catalog: Catalog, worker: Worker, settings: VapiSettings,
               admin_token: str | None = None, start_worker: bool = True) -> FastAPI:
    handler = ToolHandler(store, catalog, on_new_order=worker.wake)

    @asynccontextmanager
    async def lifespan(_app):
        if start_worker:
            worker.start()
        yield
        worker.stop()

    app = FastAPI(title="Packing cell order service", lifespan=lifespan)

    def vapi_auth(x_vapi_secret: str | None = Header(None), authorization: str | None = Header(None)):
        _check_secret(settings.server_secret, x_vapi_secret, authorization)

    def admin_auth(authorization: str | None = Header(None)):
        _check_secret(admin_token, None, authorization)

    @app.post("/vapi/webhook", dependencies=[Depends(vapi_auth)])
    async def vapi_webhook(request: Request):
        try:
            body = await request.json()
        except Exception:  # noqa: BLE001
            return {}
        return handler.handle_message(body.get("message") or {})

    @app.get("/health")
    def health():
        return {"ok": True, "worker_paused": worker.paused, "current_order": worker.current,
                "queued": len(store.list("queued")), "vapi_calls_enabled": settings.can_call}

    @app.get("/orders", dependencies=[Depends(admin_auth)])
    def list_orders(status: str | None = None, limit: int = 100):
        return [o.to_dict() for o in store.list(status, limit=limit)]

    @app.post("/orders", dependencies=[Depends(admin_auth)], status_code=201)
    def place_order(body: OrderIn):
        try:
            skus = parse_items(body.items, catalog)
            phone = normalize_phone(body.phone)
        except OrderError as e:
            raise HTTPException(422, str(e)) from e
        order, _ = store.create(phone, skus, body.customer_name, "api")
        worker.wake()
        return {**order.to_dict(), "queue_position": store.queue_position(order.id)}

    @app.get("/orders/{order_id}", dependencies=[Depends(admin_auth)])
    def get_order(order_id: int):
        o = store.get(order_id)
        if not o:
            raise HTTPException(404, "no such order")
        return {**o.to_dict(), "queue_position": store.queue_position(o.id)}

    @app.post("/orders/{order_id}/cancel", dependencies=[Depends(admin_auth)])
    def cancel_order(order_id: int):
        try:
            return store.cancel(order_id).to_dict()
        except OrderError as e:
            raise HTTPException(409, str(e)) from e

    @app.post("/orders/{order_id}/retry", dependencies=[Depends(admin_auth)])
    def retry_order(order_id: int):
        o = store.get(order_id)
        if not o:
            raise HTTPException(404, "no such order")
        if o.status == "failed":
            o = store.update(order_id, "queued", error=None)
            worker.wake()
        elif o.status == "call_failed":
            worker.call_back(o)
            o = store.get(order_id)
        else:
            raise HTTPException(409, f"order is {o.status}; only failed / call_failed can be retried")
        return o.to_dict()

    @app.post("/worker/pause", dependencies=[Depends(admin_auth)])
    def pause():
        worker.paused = True
        return {"paused": True}

    @app.post("/worker/resume", dependencies=[Depends(admin_auth)])
    def resume():
        worker.paused = False
        worker.wake()
        return {"paused": False}

    return app
