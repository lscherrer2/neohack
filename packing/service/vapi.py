"""Vapi integration: tool-call handling (Vapi -> us) and callback calls (us -> Vapi).

Tool results are short spoken-English strings: the assistant reads them to the caller.
"""

from __future__ import annotations

import json
import logging
import os
from dataclasses import dataclass, field
from typing import Callable

import httpx

from .orders import Order, OrderError, OrderStore, normalize_phone

log = logging.getLogger("packing.vapi")
VAPI_API = "https://api.vapi.ai"


# ---------------------------------------------------------------- catalog
@dataclass
class Catalog:
    products: dict[str, str]  # sku -> spoken name
    max_items: int  # box slots
    available: Callable[[], dict[str, int]] = field(default=lambda: {})  # sku -> parts on the shelf

    def resolve(self, ref: str) -> str:
        r = str(ref).strip().lower()
        for sku, name in self.products.items():
            if r in (sku.lower(), name.lower()):
                return sku
        raise OrderError(f"Sorry, we don't carry '{ref}'. We have: {self.names()}.")

    def names(self) -> str:
        return ", ".join(self.products.values()) or "nothing right now"

    def describe(self, skus: list[str]) -> str:
        counts: dict[str, int] = {}
        for s in skus:
            counts[s] = counts.get(s, 0) + 1
        return ", ".join(f"{n} {self.products.get(s, s)}" for s, n in counts.items())


def parse_items(raw, catalog: Catalog) -> list[str]:
    """Accepts [{"sku"|"product": x, "quantity": n}, ...] or ["A", "A", "B"] or "A, B"."""
    if isinstance(raw, str):
        raw = [p for p in (x.strip() for x in raw.split(",")) if p]
    skus: list[str] = []
    for it in raw or []:
        if isinstance(it, dict):
            ref = it.get("sku") or it.get("product") or it.get("name")
            qty = int(it.get("quantity") or 1)
        else:
            ref, qty = it, 1
        if qty < 1:
            continue
        skus += [catalog.resolve(ref)] * qty
    if not skus:
        raise OrderError("What would you like to order?")
    if len(skus) > catalog.max_items:
        raise OrderError(f"One box holds up to {catalog.max_items} items; that order has {len(skus)}.")
    return skus


# ---------------------------------------------------------------- tool definitions (paste into Vapi)
def tool_definitions(server_url: str) -> list[dict]:
    server = {"url": server_url}
    return [
        {"type": "function", "server": server, "function": {
            "name": "list_products",
            "description": "List the products that can be ordered and how many are in stock.",
            "parameters": {"type": "object", "properties": {}}}},
        {"type": "function", "server": server, "function": {
            "name": "place_order",
            "description": "Place an order to be packed. The customer gets a phone call back when it is ready. "
                           "Confirm the items and the callback number with the customer before calling this.",
            "parameters": {"type": "object", "properties": {
                "customer_name": {"type": "string", "description": "Customer's name"},
                "phone": {"type": "string", "description": "Callback number. Omit to use the number they called from."},
                "items": {"type": "array", "description": "Products and quantities", "items": {
                    "type": "object", "properties": {
                        "product": {"type": "string", "description": "Product name or SKU from list_products"},
                        "quantity": {"type": "integer", "minimum": 1}},
                    "required": ["product"]}}},
                "required": ["items"]}}},
        {"type": "function", "server": server, "function": {
            "name": "check_order",
            "description": "Get the status of an order. Without order_id, returns the caller's latest order.",
            "parameters": {"type": "object", "properties": {"order_id": {"type": "integer"}}}}},
        {"type": "function", "server": server, "function": {
            "name": "cancel_order",
            "description": "Cancel an order that hasn't started packing yet.",
            "parameters": {"type": "object", "properties": {"order_id": {"type": "integer"}},
                           "required": ["order_id"]}}},
    ]


# ---------------------------------------------------------------- inbound: tool calls
STATUS_SPEECH = {
    "queued": "is in line to be packed",
    "packing": "is being packed right now",
    "packed": "is packed and we're about to call you",
    "calling": "is packed; we're calling you now",
    "called": "is packed and ready",
    "failed": "ran into a problem while packing; someone will follow up",
    "call_failed": "is packed and ready, but we couldn't reach you by phone",
    "cancelled": "was cancelled",
}


class ToolHandler:
    def __init__(self, store: OrderStore, catalog: Catalog, on_new_order: Callable[[], None] = lambda: None):
        self.store, self.catalog, self.on_new_order = store, catalog, on_new_order

    def handle_message(self, message: dict) -> dict:
        """Vapi server message -> response body. Unknown message types are just acknowledged."""
        mtype = message.get("type")
        if mtype != "tool-calls":
            log.info("vapi event %s (call %s)", mtype, (message.get("call") or {}).get("id"))
            return {}
        call = message.get("call") or {}
        caller = ((message.get("customer") or call.get("customer") or {}).get("number"))
        results = []
        for tc in message.get("toolCallList") or []:
            fn = tc.get("function") or {}
            args = fn.get("arguments") or {}
            if isinstance(args, str):
                try:
                    args = json.loads(args or "{}")
                except json.JSONDecodeError:
                    args = {}
            try:
                text = self.dispatch(fn.get("name"), args, caller=caller, call_id=call.get("id"), tool_call_id=tc.get("id"))
                results.append({"toolCallId": tc.get("id"), "result": text})
            except OrderError as e:
                results.append({"toolCallId": tc.get("id"), "result": str(e)})  # speakable, not a system error
            except Exception as e:  # noqa: BLE001
                log.exception("tool %s failed", fn.get("name"))
                results.append({"toolCallId": tc.get("id"), "error": f"internal error: {e}"})
        return {"results": results}

    def dispatch(self, name: str, args: dict, caller: str | None = None, call_id: str | None = None,
                 tool_call_id: str | None = None) -> str:
        if name == "list_products":
            return self.list_products()
        if name == "place_order":
            return self.place_order(args, caller, call_id, tool_call_id)
        if name == "check_order":
            return self.check_order(args.get("order_id"), caller)
        if name == "cancel_order":
            return self.cancel_order(args.get("order_id"), caller)
        raise OrderError(f"I don't know how to '{name}'.")

    def _in_stock(self) -> dict[str, int]:
        avail, reserved = self.catalog.available(), self.store.reserved()
        return {s: max(0, avail.get(s, 0) - reserved.get(s, 0)) for s in self.catalog.products}

    def list_products(self) -> str:
        stock = self._in_stock()
        parts = [f"{name} ({stock.get(sku, 0)} in stock)" for sku, name in self.catalog.products.items()]
        return f"We have: {', '.join(parts)}. One box holds up to {self.catalog.max_items} items."

    def place_order(self, args: dict, caller: str | None, call_id: str | None, tool_call_id: str | None) -> str:
        if existing := self.store.by_tool_call(tool_call_id):  # Vapi retry: same answer, no new order
            return self._placed_text(existing)
        skus = parse_items(args.get("items"), self.catalog)
        phone = normalize_phone(args.get("phone") or caller)
        stock = self._in_stock()
        need: dict[str, int] = {}
        for s in skus:
            need[s] = need.get(s, 0) + 1
        short = [f"{self.catalog.products[s]} (only {stock.get(s, 0)} left)" for s, n in need.items() if n > stock.get(s, 0)]
        if short:
            raise OrderError("Sorry, not enough stock: " + ", ".join(short) + ".")
        order, created = self.store.create(phone, skus, args.get("customer_name"), "vapi", call_id, tool_call_id)
        if created:
            self.on_new_order()
        return self._placed_text(order)

    def _placed_text(self, order: Order) -> str:
        pos = self.store.queue_position(order.id)
        line = f"You're number {pos} in line." if pos else ""
        return (f"Order number {order.id} is placed: {self.catalog.describe(order.items)}. {line} "
                f"We'll call you at {order.phone} as soon as it's packed.").replace("  ", " ")

    def _find(self, order_id, caller: str | None) -> Order:
        if order_id is not None:
            o = self.store.get(int(order_id))
            if not o:
                raise OrderError(f"I couldn't find order {order_id}.")
            return o
        if caller:
            mine = self.store.list(phone=normalize_phone(caller), limit=1)
            if mine:
                return mine[0]
        raise OrderError("Which order number is it?")

    def check_order(self, order_id, caller: str | None) -> str:
        o = self._find(order_id, caller)
        pos = self.store.queue_position(o.id)
        extra = f" You're number {pos} in line." if pos else ""
        return f"Order {o.id} ({self.catalog.describe(o.items)}) {STATUS_SPEECH.get(o.status, o.status)}.{extra}"

    def cancel_order(self, order_id, caller: str | None) -> str:
        if order_id is None:
            raise OrderError("Which order number should I cancel?")
        o = self.store.cancel(int(order_id))
        return f"Order {o.id} is cancelled."


# ---------------------------------------------------------------- outbound: callback calls
@dataclass
class VapiSettings:
    api_key: str | None = None
    phone_number_id: str | None = None
    callback_assistant_id: str | None = None
    server_secret: str | None = None  # what Vapi sends us (X-Vapi-Secret or Bearer)

    @classmethod
    def from_env(cls) -> "VapiSettings":
        return cls(os.getenv("VAPI_API_KEY"), os.getenv("VAPI_PHONE_NUMBER_ID"),
                   os.getenv("VAPI_CALLBACK_ASSISTANT_ID"), os.getenv("VAPI_SERVER_SECRET"))

    @property
    def can_call(self) -> bool:
        return bool(self.api_key and self.phone_number_id and self.callback_assistant_id)


class VapiNotifier:
    """Places the 'your order is ready' call. Without credentials it only logs (dry run)."""

    def __init__(self, settings: VapiSettings, catalog: Catalog, client: httpx.Client | None = None):
        self.s, self.catalog = settings, catalog
        self.client = client or httpx.Client(base_url=VAPI_API, timeout=20)

    def payload(self, order: Order) -> dict:
        items = self.catalog.describe(order.items)
        return {
            "assistantId": self.s.callback_assistant_id,
            "phoneNumberId": self.s.phone_number_id,
            "customer": {"number": order.phone, **({"name": order.customer_name} if order.customer_name else {})},
            "assistantOverrides": {"variableValues": {
                "order_id": str(order.id),
                "customer_name": order.customer_name or "",
                "items": items,
                "box_id": order.box_id or "",
            }},
        }

    def notify(self, order: Order) -> str:
        """Returns the Vapi call id (or 'dry-run'). Raises on failure."""
        body = self.payload(order)
        if not self.s.can_call:
            log.warning("DRY RUN (Vapi keys not set) would POST /call: %s", json.dumps(body))
            return "dry-run"
        r = self.client.post("/call", json=body, headers={"Authorization": f"Bearer {self.s.api_key}"})
        if r.status_code >= 300:
            raise RuntimeError(f"Vapi /call failed {r.status_code}: {r.text[:300]}")
        return r.json().get("id", "")
