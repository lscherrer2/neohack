"""Persistent order queue (SQLite).

queued -> [ready] -> packing -> packed -> calling -> called
   ('ready' = actioned by a person; used when packing is started manually, see claim_next(from_status))
                  \\-> failed      \\-> call_failed
queued -> cancelled
"""

from __future__ import annotations

import json
import re
import sqlite3
import threading
from collections.abc import Callable
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path

STATUSES = ("queued", "ready", "packing", "packed", "calling", "called", "failed", "call_failed", "cancelled")

SCHEMA = """
CREATE TABLE IF NOT EXISTS orders (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    customer_name TEXT,
    phone TEXT NOT NULL,
    items TEXT NOT NULL,              -- JSON list of SKUs, one per box slot
    status TEXT NOT NULL,
    source TEXT NOT NULL,             -- 'vapi' | 'api'
    vapi_call_id TEXT,                -- the inbound call that placed it
    tool_call_id TEXT UNIQUE,         -- idempotency key for retried Vapi tool calls
    box_id TEXT,
    callback_call_id TEXT,
    error TEXT,
    created_at TEXT NOT NULL,
    updated_at TEXT NOT NULL
);
CREATE INDEX IF NOT EXISTS orders_status ON orders(status, id);

-- Per-unit stock reservations: each item of an open order owns one specific unit (tray pocket / part slot).
-- slot NULL = short (no free unit right now). UNIQUE(slot): a unit belongs to at most one order.
CREATE TABLE IF NOT EXISTS reservations (
    order_id INTEGER NOT NULL,
    item INTEGER NOT NULL,            -- index into the order's items
    sku TEXT NOT NULL,
    slot TEXT UNIQUE,
    PRIMARY KEY (order_id, item)
);
-- Released / consumed automatically when an order leaves the open states or is deleted.
CREATE TRIGGER IF NOT EXISTS reservations_release AFTER UPDATE OF status ON orders
    WHEN NEW.status NOT IN ('queued', 'ready', 'packing', 'failed')
    BEGIN DELETE FROM reservations WHERE order_id = NEW.id; END;
CREATE TRIGGER IF NOT EXISTS reservations_delete AFTER DELETE ON orders
    BEGIN DELETE FROM reservations WHERE order_id = OLD.id; END;
"""

OPEN = ("queued", "ready", "packing", "failed")  # orders that hold reservations

# stock() -> {sku: [slots holding an available unit, preferred first]}
Stock = Callable[[], dict[str, list[str]]]


class OrderError(ValueError):
    """Bad request from the caller (unknown product, too many items, ...). Message is speakable."""


@dataclass
class Order:
    id: int
    customer_name: str | None
    phone: str
    items: list[str]
    status: str
    source: str
    vapi_call_id: str | None
    tool_call_id: str | None
    box_id: str | None
    callback_call_id: str | None
    error: str | None
    created_at: str
    updated_at: str
    slots: list[str | None] = field(default_factory=list)  # reserved unit per item (None = short); [] = none

    def to_dict(self) -> dict:
        return dict(self.__dict__)


def _now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="seconds")


def normalize_phone(raw: str | None) -> str:
    """Best-effort E.164 (+15551234567). US 10-digit numbers get +1."""
    if not raw:
        raise OrderError("I need a phone number to call you back.")
    digits = re.sub(r"\D", "", raw)
    if raw.strip().startswith("+") and 8 <= len(digits) <= 15:
        return "+" + digits
    if len(digits) == 10:
        return "+1" + digits
    if len(digits) == 11 and digits.startswith("1"):
        return "+" + digits
    raise OrderError(f"'{raw}' doesn't look like a valid phone number.")


class OrderStore:
    def __init__(self, path: str | Path):
        if str(path) != ":memory:":
            Path(path).parent.mkdir(parents=True, exist_ok=True)
        self.db = sqlite3.connect(str(path), isolation_level=None, check_same_thread=False)
        self.db.row_factory = sqlite3.Row
        self.db.executescript(SCHEMA)
        self.lock = threading.Lock()  # server threads + worker thread share one connection

    def _row(self, r: sqlite3.Row) -> Order:
        d = dict(r)
        d["items"] = json.loads(d["items"])
        rows = self.db.execute("SELECT item, slot FROM reservations WHERE order_id=? ORDER BY item", (d["id"],))
        d["slots"] = [x["slot"] for x in rows]
        return Order(**d)

    def create(self, phone: str, items: list[str], customer_name: str | None = None, source: str = "api",
               vapi_call_id: str | None = None, tool_call_id: str | None = None,
               stock: Stock | None = None) -> tuple[Order, bool]:
        """Returns (order, created). If tool_call_id was seen before, returns that order, created=False.
        `stock`: reserve one specific unit per item (lowest-numbered free first), all or nothing — OrderError
        "not enough stock" if any item has no free unit. Atomic across threads and processes."""
        with self.lock:
            if tool_call_id:
                r = self.db.execute("SELECT * FROM orders WHERE tool_call_id=?", (tool_call_id,)).fetchone()
                if r:
                    return self._row(r), False
            now = _now()
            self.db.execute("BEGIN IMMEDIATE")  # also locks out other processes until COMMIT
            try:
                slots = self._allocate(items, stock()) if stock else None
                cur = self.db.execute(
                    "INSERT INTO orders (customer_name, phone, items, status, source, vapi_call_id, tool_call_id,"
                    " created_at, updated_at) VALUES (?,?,?, 'queued', ?,?,?,?,?)",
                    (customer_name, phone, json.dumps(items), source, vapi_call_id, tool_call_id, now, now))
                if slots is not None:
                    self.db.executemany("INSERT INTO reservations (order_id, item, sku, slot) VALUES (?,?,?,?)",
                                        [(cur.lastrowid, i, sku, slot) for i, (sku, slot) in enumerate(zip(items, slots))])
                self.db.execute("COMMIT")
            except BaseException:
                self.db.execute("ROLLBACK")
                raise
            return self.get(cur.lastrowid), True

    # ---------- per-unit reservations ----------
    def _taken(self) -> set[str]:
        return {r["slot"] for r in self.db.execute("SELECT slot FROM reservations WHERE slot IS NOT NULL")}

    def _allocate(self, items: list[str], stock: dict[str, list[str]]) -> list[str]:
        """One free unit per item, preferred first. OrderError (speakable) listing what's short."""
        taken, out, short = self._taken(), [], {}
        for sku in items:
            slot = next((s for s in stock.get(sku, []) if s not in taken), None)
            if slot is None:
                short[sku] = short.get(sku, 0) + 1
            else:
                taken.add(slot)
                out.append(slot)
        if short:
            free = {sku: sum(s not in self._taken() for s in stock.get(sku, [])) for sku in short}
            raise OrderError("Sorry, not enough stock: " + ", ".join(
                f"{sku} (only {free[sku]} left)" for sku in short) + ".")
        return out

    def reserved_slots(self) -> dict[str, int]:
        """slot -> order id, for every reserved unit."""
        return {r["slot"]: r["order_id"] for r in
                self.db.execute("SELECT slot, order_id FROM reservations WHERE slot IS NOT NULL")}

    def locked_slots(self) -> set[str]:
        """Units reserved by an order that is being packed: nobody may change them from the dashboard."""
        return {r["slot"] for r in self.db.execute(
            "SELECT r.slot FROM reservations r JOIN orders o ON o.id = r.order_id"
            " WHERE o.status = 'packing' AND r.slot IS NOT NULL")}

    def release_slot(self, slot: str, stock: dict[str, list[str]]) -> int | None:
        """A unit is gone (marked empty): move its reservation to another free unit of the same sku, or make
        that item short. Returns the affected order id (None if the unit wasn't reserved)."""
        with self.lock:
            r = self.db.execute("SELECT order_id, item, sku FROM reservations WHERE slot=?", (slot,)).fetchone()
            if not r:
                return None
            taken = self._taken() - {slot}
            new = next((s for s in stock.get(r["sku"], []) if s not in taken and s != slot), None)
            self.db.execute("UPDATE reservations SET slot=? WHERE order_id=? AND item=?", (new, r["order_id"], r["item"]))
            return r["order_id"]

    def fill_short(self, stock: dict[str, list[str]]) -> list[int]:
        """Give short items (slot NULL) a free unit where possible, oldest order first; also backfills open
        orders that have no reservation rows yet (made before reservations existed). Returns order ids changed."""
        changed: list[int] = []
        with self.lock:
            ph = ",".join("?" * len(OPEN))
            for o in self.db.execute(f"SELECT id, items FROM orders WHERE status IN ({ph}) AND id NOT IN"
                                     " (SELECT order_id FROM reservations) ORDER BY id", OPEN).fetchall():
                self.db.executemany("INSERT INTO reservations (order_id, item, sku, slot) VALUES (?,?,?,NULL)",
                                    [(o["id"], i, sku) for i, sku in enumerate(json.loads(o["items"]))])
            taken = self._taken()
            for r in self.db.execute("SELECT order_id, item, sku FROM reservations WHERE slot IS NULL"
                                     " ORDER BY order_id, item").fetchall():
                new = next((s for s in stock.get(r["sku"], []) if s not in taken), None)
                if new:
                    taken.add(new)
                    self.db.execute("UPDATE reservations SET slot=? WHERE order_id=? AND item=?",
                                    (new, r["order_id"], r["item"]))
                    if r["order_id"] not in changed:
                        changed.append(r["order_id"])
        return changed

    def by_tool_call(self, tool_call_id: str | None) -> Order | None:
        if not tool_call_id:
            return None
        r = self.db.execute("SELECT * FROM orders WHERE tool_call_id=?", (tool_call_id,)).fetchone()
        return self._row(r) if r else None

    def get(self, order_id: int) -> Order | None:
        r = self.db.execute("SELECT * FROM orders WHERE id=?", (order_id,)).fetchone()
        return self._row(r) if r else None

    def list(self, status: str | None = None, phone: str | None = None, limit: int = 100) -> list[Order]:
        q, a = "SELECT * FROM orders WHERE 1=1", []
        if status:
            q, a = q + " AND status=?", [*a, status]
        if phone:
            q, a = q + " AND phone=?", [*a, phone]
        rows = self.db.execute(q + " ORDER BY id DESC LIMIT ?", (*a, limit))
        return [self._row(r) for r in rows]

    def queue_position(self, order_id: int) -> int | None:
        """1 = next to be packed. None if not queued."""
        o = self.get(order_id)
        if not o or o.status not in ("queued", "ready"):
            return None
        return self.db.execute("SELECT COUNT(*) FROM orders WHERE status IN ('queued','ready') AND id<=?",
                               (order_id,)).fetchone()[0]

    def reserved(self) -> dict[str, int]:
        """SKU counts in orders not yet packed (queued, ready or packing) — reserved units plus short ones."""
        out: dict[str, int] = {}
        for r in self.db.execute("SELECT items FROM orders WHERE status IN ('queued','ready','packing')"):
            for sku in json.loads(r["items"]):
                out[sku] = out.get(sku, 0) + 1
        return out

    def claim_next(self, from_status: str = "queued") -> Order | None:
        """Atomically move the oldest order in `from_status` ('queued', or 'ready' for manual starts) to 'packing'."""
        with self.lock:
            r = self.db.execute("SELECT id FROM orders WHERE status=? ORDER BY id LIMIT 1", (from_status,)).fetchone()
            if not r:
                return None
            self.db.execute("UPDATE orders SET status='packing', updated_at=? WHERE id=? AND status=?",
                            (_now(), r["id"], from_status))
            return self.get(r["id"])

    def update(self, order_id: int, status: str | None = None, **fields) -> Order:
        allowed = {"box_id", "callback_call_id", "error"}
        if bad := set(fields) - allowed:
            raise ValueError(f"can't update {bad}")
        if status and status not in STATUSES:
            raise ValueError(status)
        sets, args = ["updated_at=?"], [_now()]
        if status:
            sets.append("status=?")
            args.append(status)
        for k, v in fields.items():
            sets.append(f"{k}=?")
            args.append(v)
        with self.lock:
            self.db.execute(f"UPDATE orders SET {', '.join(sets)} WHERE id=?", (*args, order_id))
        return self.get(order_id)

    def action(self, order_id: int) -> Order:
        """A person releases a received order to the robot: queued -> ready."""
        with self.lock:
            o = self.get(order_id)
            if not o:
                raise OrderError(f"No order {order_id}.")
            if o.status != "queued":
                raise OrderError(f"Order {order_id} is {o.status}; only received orders can be actioned.")
            self.db.execute("UPDATE orders SET status='ready', updated_at=? WHERE id=?", (_now(), order_id))
        return self.get(order_id)

    DELETABLE = ("queued", "ready", "failed", "cancelled")

    def delete(self, order_id: int) -> Order:
        """Remove an order that isn't being packed or already packed (e.g. a broken test order)."""
        with self.lock:
            o = self.get(order_id)
            if not o:
                raise OrderError(f"No order {order_id}.")
            if o.status not in self.DELETABLE:
                raise OrderError(f"Order {order_id} is {o.status} and can't be deleted.")
            self.db.execute("DELETE FROM orders WHERE id=?", (order_id,))
        return o

    def delete_all(self) -> list[Order]:
        """Remove every deletable order (keeps ones being packed or packed). Returns the removed orders."""
        with self.lock:
            ph = ",".join("?" * len(self.DELETABLE))
            gone = [self._row(r) for r in self.db.execute(
                f"SELECT * FROM orders WHERE status IN ({ph}) ORDER BY id", self.DELETABLE)]
            self.db.execute(f"DELETE FROM orders WHERE status IN ({ph})", self.DELETABLE)
        return gone

    def cancel(self, order_id: int, phone: str | None = None) -> Order:
        with self.lock:
            o = self.get(order_id)
            if not o or (phone and o.phone != phone):
                raise OrderError(f"I couldn't find order {order_id}.")
            if o.status not in ("queued", "ready"):
                raise OrderError(f"Order {order_id} is already {o.status.replace('_', ' ')} and can't be cancelled.")
            self.db.execute("UPDATE orders SET status='cancelled', updated_at=? WHERE id=?", (_now(), order_id))
        return self.get(order_id)

    def recover(self) -> list[int]:
        """On startup: orders stuck in 'packing'/'calling' from a crash. Packing ones are re-queued (the cell
        resumes from inventory); calling ones go to call_failed so an operator can retry."""
        with self.lock:
            stuck = [r["id"] for r in self.db.execute("SELECT id FROM orders WHERE status IN ('packing','calling')")]
            self.db.execute("UPDATE orders SET status='queued', updated_at=? WHERE status='packing'", (_now(),))
            self.db.execute("UPDATE orders SET status='call_failed', error='interrupted during callback',"
                            " updated_at=? WHERE status='calling'", (_now(),))
        return stuck
