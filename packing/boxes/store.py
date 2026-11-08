"""Box records (in the orders database, next to the orders they hold).

A box is `available` (calibrated, empty, at its supply spot on the left), `in_use` (taken by an order; at the
pack spot in the middle while it's filled) or `packed` (done: at its output spot = the supply spot mirrored to
the right). `location` says where it physically is: supply | pack | output. Boxes only move left → right; a packed
box is deleted when it's picked up, new boxes come in only through Calibration → "Add box". For testing, `reset`
puts a box record back to empty at its calibrated supply spot (after a human put the box back there).
"""

from __future__ import annotations

import json
from dataclasses import dataclass

import numpy as np

from ..service.orders import OrderStore, _now

SCHEMA = """
CREATE TABLE IF NOT EXISTS boxes (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    state TEXT NOT NULL,               -- available | in_use | packed
    location TEXT NOT NULL,            -- supply | pack | output
    order_id INTEGER,
    supply TEXT NOT NULL,              -- JSON 4x4 box pose at its supply spot (robot frame, m)
    tool_in_box TEXT NOT NULL,         -- JSON 4x4 grasp: tool pose in the box frame (from the calibration)
    joints TEXT NOT NULL,              -- JSON joint pose at the calibration grasp
    report TEXT,                       -- JSON capture checks
    created_at TEXT NOT NULL,
    updated_at TEXT NOT NULL
);
"""

STATES = ("available", "in_use", "packed")
LOCATIONS = ("supply", "pack", "output")


@dataclass
class Box:
    id: int
    state: str
    location: str
    order_id: int | None
    supply: np.ndarray
    tool_in_box: np.ndarray
    joints: dict[str, float]
    report: dict
    created_at: str
    updated_at: str

    def to_dict(self) -> dict:
        return {"id": self.id, "state": self.state, "location": self.location, "order_id": self.order_id,
                "supply": self.supply.round(6).tolist(), "report": self.report,
                "created_at": self.created_at, "updated_at": self.updated_at}


class BoxError(ValueError):
    """Not allowed (message shown to the user)."""


class BoxStore:
    """Shares the OrderStore's connection and lock."""

    def __init__(self, orders: OrderStore):
        self.orders, self.db, self.lock = orders, orders.db, orders.lock
        self.db.executescript(SCHEMA)

    def _row(self, r) -> Box:
        return Box(r["id"], r["state"], r["location"], r["order_id"], np.array(json.loads(r["supply"])),
                   np.array(json.loads(r["tool_in_box"])), json.loads(r["joints"]), json.loads(r["report"] or "{}"),
                   r["created_at"], r["updated_at"])

    def list(self) -> list[Box]:
        return [self._row(r) for r in self.db.execute("SELECT * FROM boxes ORDER BY id")]

    def get(self, box_id: int) -> Box:
        r = self.db.execute("SELECT * FROM boxes WHERE id=?", (box_id,)).fetchone()
        if not r:
            raise BoxError(f"no box {box_id}")
        return self._row(r)

    def add(self, supply: np.ndarray, tool_in_box: np.ndarray, joints: dict[str, float], report: dict) -> Box:
        now = _now()
        with self.lock:
            cur = self.db.execute(
                "INSERT INTO boxes (state, location, supply, tool_in_box, joints, report, created_at, updated_at)"
                " VALUES ('available', 'supply', ?,?,?,?,?,?)",
                (json.dumps(np.asarray(supply).tolist()), json.dumps(np.asarray(tool_in_box).tolist()),
                 json.dumps(joints), json.dumps(report), now, now))
        return self.get(cur.lastrowid)

    def remove(self, box_id: int) -> Box:
        b = self.get(box_id)
        if b.state == "in_use":
            r = self.db.execute("SELECT status FROM orders WHERE id=?", (b.order_id,)).fetchone()
            if r and r["status"] == "packing":
                raise BoxError(f"box {box_id} is in use by order {b.order_id}, which is being packed")
        with self.lock:
            self.db.execute("DELETE FROM boxes WHERE id=?", (box_id,))
        return b

    def reset(self, box_id: int) -> Box:
        """Back to the start (testing): empty, at its calibrated supply spot, no order — a human has put the box
        back where it was when it was added, so no re-calibration is needed. Not while its order is being packed."""
        b = self.get(box_id)
        if b.state == "in_use":
            r = self.db.execute("SELECT status FROM orders WHERE id=?", (b.order_id,)).fetchone()
            if r and r["status"] == "packing":
                raise BoxError(f"box {box_id} is in use by order {b.order_id}, which is being packed")
        return self.update(box_id, location="supply", state="available", order_id=None)

    def peek(self, order_id: int) -> Box:
        """The box `take` would give this order, without assigning it (dry runs)."""
        r = (self.db.execute("SELECT * FROM boxes WHERE order_id=? AND state='in_use'", (order_id,)).fetchone()
             or self.db.execute("SELECT * FROM boxes WHERE state='available' AND location='supply' ORDER BY id"
                                " LIMIT 1").fetchone())
        if not r:
            raise BoxError("no empty box available — calibrate one on the Calibration tab")
        return self._row(r)

    def take(self, order_id: int) -> Box:
        """The order's box: the one it already has, else the first available box."""
        with self.lock:
            r = self.db.execute("SELECT * FROM boxes WHERE order_id=? AND state='in_use'", (order_id,)).fetchone()
            if r:
                return self._row(r)
            r = self.db.execute("SELECT * FROM boxes WHERE state='available' AND location='supply'"
                                " ORDER BY id LIMIT 1").fetchone()
            if not r:
                raise BoxError("no empty box available — calibrate one on the Calibration tab")
            self.db.execute("UPDATE boxes SET state='in_use', order_id=?, updated_at=? WHERE id=?",
                            (order_id, _now(), r["id"]))
        return self.get(r["id"])

    def recalibrate(self, box_id: int, supply: np.ndarray, tool_in_box: np.ndarray) -> Box:
        """New supply pose / grasp for a box (re-derived from its capture joints, e.g. after a kinematics change)."""
        with self.lock:
            self.db.execute("UPDATE boxes SET supply=?, tool_in_box=?, updated_at=? WHERE id=?",
                            (json.dumps(np.asarray(supply).tolist()), json.dumps(np.asarray(tool_in_box).tolist()),
                             _now(), box_id))
        return self.get(box_id)

    def update(self, box_id: int, location: str | None = None, state: str | None = None,
               order_id: int | None | bool = False) -> Box:
        if location and location not in LOCATIONS or state and state not in STATES:
            raise ValueError((location, state))
        sets, args = ["updated_at=?"], [_now()]
        for col, v in (("location", location), ("state", state)):
            if v:
                sets.append(f"{col}=?")
                args.append(v)
        if order_id is not False:
            sets.append("order_id=?")
            args.append(order_id)
        with self.lock:
            self.db.execute(f"UPDATE boxes SET {', '.join(sets)} WHERE id=?", (*args, box_id))
        return self.get(box_id)
