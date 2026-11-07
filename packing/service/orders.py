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
from dataclasses import dataclass
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
"""


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
        return Order(**d)

    def create(self, phone: str, items: list[str], customer_name: str | None = None, source: str = "api",
               vapi_call_id: str | None = None, tool_call_id: str | None = None) -> tuple[Order, bool]:
        """Returns (order, created). If tool_call_id was seen before, returns that order, created=False."""
        with self.lock:
            if tool_call_id:
                r = self.db.execute("SELECT * FROM orders WHERE tool_call_id=?", (tool_call_id,)).fetchone()
                if r:
                    return self._row(r), False
            now = _now()
            cur = self.db.execute(
                "INSERT INTO orders (customer_name, phone, items, status, source, vapi_call_id, tool_call_id,"
                " created_at, updated_at) VALUES (?,?,?, 'queued', ?,?,?,?,?)",
                (customer_name, phone, json.dumps(items), source, vapi_call_id, tool_call_id, now, now))
            return self.get(cur.lastrowid), True

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
        """SKU counts in orders not yet packed (queued or packing)."""
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
