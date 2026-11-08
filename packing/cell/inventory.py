"""Persistent parts inventory (SQLite): one row per unit in a tray pocket, plus an event log.
Humans mark pockets full/empty (dashboard); packing marks units packed. Boxes are not tracked here:
they live in the orders DB `boxes` table (packing/boxes/store.py).

Part status:  available -> in_gripper -> packed        (or -> missing if a grasp failed / pocket emptied)
"""

from __future__ import annotations

import sqlite3
from dataclasses import dataclass, fields
from datetime import datetime
from pathlib import Path

SCHEMA = """
CREATE TABLE IF NOT EXISTS parts (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    sku TEXT NOT NULL,
    location TEXT NOT NULL,
    status TEXT NOT NULL CHECK (status IN ('available','in_gripper','packed','missing')),
    box_id TEXT,
    box_slot INTEGER,
    updated_at TEXT NOT NULL
);
CREATE TABLE IF NOT EXISTS events (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    ts TEXT NOT NULL,
    kind TEXT NOT NULL,
    part_id INTEGER,
    box_id TEXT,
    detail TEXT
);
"""


@dataclass
class Part:
    """A part row; `location` is a slot name (or a box id for parts packed by the old Cell path)."""

    id: int
    sku: str
    location: str
    status: str
    box_id: str | None
    box_slot: int | None


def _part(row: sqlite3.Row) -> Part:
    return Part(**{f.name: row[f.name] for f in fields(Part)})


def _now() -> str:
    return datetime.now().isoformat(timespec="seconds")


class InventoryError(RuntimeError):
    """An inventory change that doesn't make sense (unknown id, wrong status)."""


# Event row: (kind, part_id, box_id, detail)
Event = tuple[str, int | None, str | None, str]
Statement = tuple[str, tuple]


class Inventory:
    """SQLite-backed record of parts and an event log. Each change is one transaction."""

    def __init__(self, path: str | Path):
        if str(path) != ":memory:":
            Path(path).parent.mkdir(parents=True, exist_ok=True)
        self.db = sqlite3.connect(str(path), isolation_level=None)  # autocommit; explicit tx below
        self.db.row_factory = sqlite3.Row
        self.db.executescript(SCHEMA)

    # ---------- helpers ----------
    def _tx(self, statements: list[Statement], event: Event) -> None:
        """Run `statements` and log `event` atomically."""
        with self.db:  # BEGIN/COMMIT, rolls back on error
            self.db.execute("BEGIN")
            for sql, args in statements:
                self.db.execute(sql, args)
            self.db.execute(
                "INSERT INTO events (ts, kind, part_id, box_id, detail) VALUES (?, ?, ?, ?, ?)",
                (_now(), *event),
            )

    # ---------- queries ----------
    def part(self, part_id: int) -> Part:
        row = self.db.execute("SELECT * FROM parts WHERE id = ?", (part_id,)).fetchone()
        if not row:
            raise InventoryError(f"no part {part_id}")
        return _part(row)

    def parts(self, status: str | None = None) -> list[Part]:
        if status:
            rows = self.db.execute("SELECT * FROM parts WHERE status = ? ORDER BY id", (status,))
        else:
            rows = self.db.execute("SELECT * FROM parts ORDER BY id")
        return [_part(row) for row in rows]

    def available_part(self, location: str) -> Part | None:
        row = self.db.execute(
            """
            SELECT * FROM parts
            WHERE location = ? AND status = 'available'
            ORDER BY id
            LIMIT 1
            """,
            (location,),
        ).fetchone()
        return _part(row) if row else None

    def in_gripper(self) -> list[str]:
        """Human-readable list of every part the inventory says the gripper is holding."""
        return [f"part {p.id} ({p.sku})" for p in self.parts("in_gripper")]

    def events(self, limit: int = 20) -> list[sqlite3.Row]:
        """Most recent events, newest first."""
        return list(self.db.execute("SELECT * FROM events ORDER BY id DESC LIMIT ?", (limit,)))

    # ---------- stocking ----------
    def restock(self, part_slots: dict[str, str]) -> int:
        """Put one available part in every empty part slot (location -> sku). Call after physically refilling
        the slots. Returns the number of parts added."""
        added_parts = 0
        for location, sku in part_slots.items():
            if not self.available_part(location):
                self._tx(
                    [("INSERT INTO parts (sku, location, status, updated_at) VALUES (?, ?, 'available', ?)",
                      (sku, location, _now()))],
                    ("restock_part", None, None, f"{sku} @ {location}"),
                )
                added_parts += 1
        return added_parts

    def set_slot(self, location: str, sku: str, full: bool) -> bool:
        """A human says what is physically in a slot. full=True: one available `sku` part there (added if
        missing). full=False: its available part is gone (marked 'missing'). Returns True if anything changed."""
        part = self.available_part(location)
        if full and not part:
            self._tx(
                [("INSERT INTO parts (sku, location, status, updated_at) VALUES (?, ?, 'available', ?)",
                  (sku, location, _now()))],
                ("restock_part", None, None, f"{sku} @ {location}"),
            )
            return True
        if not full and part:
            self._tx(
                [("UPDATE parts SET status = 'missing', updated_at = ? WHERE id = ?", (_now(), part.id))],
                ("slot_emptied", part.id, None, location),
            )
            return True
        return False

    def full_slots(self) -> set[str]:
        """Locations holding an available part."""
        rows = self.db.execute("SELECT DISTINCT location FROM parts WHERE status = 'available'")
        return {row["location"] for row in rows}

    # ---------- transitions ----------
    def pick_part(self, part_id: int) -> None:
        """available -> in_gripper."""
        part = self.part(part_id)
        if part.status != "available":
            raise InventoryError(f"part {part_id} is {part.status}, not available")
        self._tx(
            [("UPDATE parts SET status = 'in_gripper', updated_at = ? WHERE id = ?", (_now(), part_id))],
            ("pick_part", part_id, None, part.location),
        )

    def set_part(self, part_id: int, status: str, location: str | None = None) -> None:
        """Manual correction. Box id/slot are kept only if the new status is 'packed'."""
        part = self.part(part_id)
        self._tx(
            [("""
              UPDATE parts
              SET status = ?,
                  location = ?,
                  box_id = CASE WHEN ? = 'packed' THEN box_id ELSE NULL END,
                  box_slot = CASE WHEN ? = 'packed' THEN box_slot ELSE NULL END,
                  updated_at = ?
              WHERE id = ?
              """,
              (status, location or part.location, status, status, _now(), part_id))],
            ("set_part", part_id, None, f"{part.status}->{status}"),
        )

    # ---------- reporting ----------
    def summary(self) -> str:
        """Multi-line status report: part counts, anything stuck in the gripper."""
        lines = ["PARTS"]
        counts = self.db.execute(
            "SELECT sku, status, COUNT(*) n FROM parts GROUP BY sku, status ORDER BY sku, status"
        )
        for row in counts:
            lines.append(f"  {row['sku']:<12} {row['status']:<11} {row['n']}")
        held = self.in_gripper()
        if held:
            lines.append("!! IN GRIPPER (resolve before running): " + ", ".join(held))
        return "\n".join(lines)
