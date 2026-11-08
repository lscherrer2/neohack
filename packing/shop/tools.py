"""Customer-facing order tools. Plain functions returning short spoken-style sentences; any front end
(MCP today, a voice agent later) just relays them."""

from __future__ import annotations

from ..service.orders import Order, OrderError, OrderStore, normalize_phone
from ..service.vapi import Catalog, parse_items

STATUS_TEXT = {
    "queued": "has been received and is waiting for the shop to start packing it",
    "ready": "is lined up for the robot to pack next",
    "packing": "is being packed right now",
    "packed": "is packed and ready for pickup",
    "failed": "hit a problem while packing; someone from the shop will follow up",
    "cancelled": "was cancelled",
}


class ShopTools:
    def __init__(self, store: OrderStore, catalog: Catalog):
        self.store, self.catalog = store, catalog

    def in_stock(self) -> dict[str, int]:
        avail, reserved = self.catalog.available(), self.store.reserved()
        return {s: max(0, avail.get(s, 0) - reserved.get(s, 0)) for s in self.catalog.products}

    def list_products(self) -> str:
        stock = self.in_stock()
        parts = [f"{name} ({stock.get(sku, 0)} in stock)" for sku, name in self.catalog.products.items()]
        return f"We have: {', '.join(parts)}. An order can have up to {self.catalog.max_items} items in total."

    def place_order(self, items, customer_name: str, phone: str | None = None) -> str:
        return self.create_order(items, customer_name, phone)[1]

    def create_order(self, items, customer_name: str, phone: str | None = None,
                     source: str = "mcp") -> tuple[Order, str]:
        """Place an order; returns (order, spoken confirmation). OrderError if invalid / out of stock."""
        if not (customer_name or "").strip():
            raise OrderError("What name should I put the order under?")
        skus = parse_items(items, self.catalog)
        stock = self.in_stock()
        need: dict[str, int] = {}
        for s in skus:
            need[s] = need.get(s, 0) + 1
        short = [f"{self.catalog.products[s]} (only {stock.get(s, 0)} left)" for s, n in need.items() if n > stock.get(s, 0)]
        if short:
            raise OrderError("Sorry, not enough stock: " + ", ".join(short) + ".")
        order, _ = self.store.create(normalize_phone(phone) if phone else "", skus, customer_name.strip(), source,
                                     stock=self.catalog.stock)  # reserves specific units (atomic re-check)
        pos = self.store.queue_position(order.id)
        return order, (f"Order number {order.id} for {order.customer_name} is placed: {self.catalog.describe(order.items)}. "
                f"It's number {pos} in line.")

    def _get(self, order_id) -> Order:
        o = self.store.get(int(order_id)) if order_id is not None else None
        if not o:
            raise OrderError(f"I couldn't find order {order_id}." if order_id is not None else "Which order number is it?")
        return o

    def check_order(self, order_id) -> str:
        o = self._get(order_id)
        pos = self.store.queue_position(o.id)
        extra = f" It's number {pos} in line." if pos else ""
        return f"Order {o.id} ({self.catalog.describe(o.items)}) {STATUS_TEXT.get(o.status, o.status)}.{extra}"

    def cancel_order(self, order_id) -> str:
        if order_id is None:
            raise OrderError("Which order number should I cancel?")
        return f"Order {self.store.cancel(int(order_id)).id} is cancelled."
