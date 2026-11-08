"""Phone the customer when their order is done (packed or failed), through Vapi.

POST https://api.vapi.ai/call with the follow-up assistant (VAPI_CALLBACK_ASSISTANT_ID), from our outbound
number (VAPI_PHONE_NUMBER_ID), auth VAPI_API_KEY — read from the environment, else <repo>/.env.local (never
printed). One attempt per order, no redials. Runs in a background thread so the robot's report doesn't wait.
Only `start --robot` makes one (simulated packing never calls). Prompt for the assistant: vapi_callback_prompt.md.
"""

from __future__ import annotations

import logging
import os
import re
import threading
from pathlib import Path

import httpx

from ..service.orders import Order, OrderStore
from ..service.vapi import VAPI_API, Catalog
from .events import EventLog

log = logging.getLogger(__name__)
REPO_ROOT = Path(__file__).resolve().parents[2]
ENV_KEYS = ("VAPI_API_KEY", "VAPI_PHONE_NUMBER_ID", "VAPI_CALLBACK_ASSISTANT_ID")
SHOP_NAME = "the parts shop"


def load_env(path: Path = REPO_ROOT / ".env.local", keys=ENV_KEYS) -> None:
    """Copy `keys` from a KEY=value file into os.environ (unless already set)."""
    if not path.exists():
        return
    for line in path.read_text().splitlines():
        m = re.match(r'\s*(?:export\s+)?([A-Z_]+)\s*=\s*["\']?([^"\'\s#]+)', line)
        if m and m.group(1) in keys and not os.getenv(m.group(1)):
            os.environ[m.group(1)] = m.group(2)


def first_message(order: Order, items: str) -> str:
    hi = f"Hi {order.customer_name}" if order.customer_name else "Hi there"
    if order.status == "packed":
        return f"{hi}, this is {SHOP_NAME}. Your order with {items} is ready for pickup."
    return (f"{hi}, this is {SHOP_NAME}. Sorry, we couldn't pack your order with {items}, so it isn't ready. "
            "Please call us back and we'll sort it out.")


class Callbacks:
    def __init__(self, store: OrderStore, catalog: Catalog, events: EventLog, api_key: str, phone_number_id: str,
                 assistant_id: str, client: httpx.Client | None = None, background: bool = True):
        self.store, self.catalog, self.events = store, catalog, events
        self.api_key, self.phone_number_id, self.assistant_id = api_key, phone_number_id, assistant_id
        self.client = client or httpx.Client(base_url=VAPI_API, timeout=20)
        self.background = background

    @classmethod
    def from_env(cls, store: OrderStore, catalog: Catalog, events: EventLog) -> "Callbacks | None":
        """None (with a log line) if any Vapi setting is missing."""
        load_env()
        vals = [os.getenv(k) for k in ENV_KEYS]
        if not all(vals):
            missing = [k for k, v in zip(ENV_KEYS, vals) if not v]
            log.warning("Customer callbacks off: %s not set (env or .env.local)", ", ".join(missing))
            return None
        return cls(store, catalog, events, *vals)

    def payload(self, order: Order) -> dict:
        items = self.catalog.describe(order.items)
        return {
            "assistantId": self.assistant_id,
            "phoneNumberId": self.phone_number_id,
            "customer": {"number": order.phone, **({"name": order.customer_name} if order.customer_name else {})},
            "assistantOverrides": {
                "firstMessage": first_message(order, items),
                "variableValues": {
                    "order_id": str(order.id),
                    "customer_name": order.customer_name or "",
                    "items": items,
                    "status": "ready" if order.status == "packed" else "failed",
                },
            },
        }

    def order_finished(self, order: Order) -> None:
        """Call the customer about a packed / failed order (once; no phone number = no call)."""
        if order.status not in ("packed", "failed"):
            return
        if not order.phone:
            self.store.update(order.id, callback_status="skipped: no phone number")
            self.events.record("system", f"Order {order.id}: no phone number, no callback", order_id=order.id)
            return
        self.store.update(order.id, callback_status="calling")
        if self.background:
            threading.Thread(target=self._call, args=(order,), name=f"callback-{order.id}", daemon=True).start()
        else:
            self._call(order)

    def _call(self, order: Order) -> None:
        what = "ready" if order.status == "packed" else "failed"
        try:
            r = self.client.post("/call", json=self.payload(order),
                                 headers={"Authorization": f"Bearer {self.api_key}"})
            if r.status_code >= 300:
                raise RuntimeError(f"Vapi {r.status_code}: {r.text[:200]}")
            call_id = r.json().get("id", "")
        except Exception as e:  # noqa: BLE001 — a failed call never breaks packing
            log.warning("Callback for order %s failed: %s", order.id, e)
            self.store.update(order.id, callback_status=f"failed: {e}"[:300])
            self.events.record("error", f"Order {order.id}: callback to {order.phone} failed: {e}",
                               order_id=order.id)
            return
        self.store.update(order.id, callback_call_id=call_id, callback_status="called")
        self.events.record("order", f"Order {order.id}: calling {order.phone} ({what})", order_id=order.id,
                           call_id=call_id)
