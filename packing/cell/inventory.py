"""Persistent inventory (SQLite). Every physical step is committed before/after it happens, so after a
crash the database says exactly what was in the gripper, what was packed, and what is still waiting.

Part status:  available -> in_gripper -> packed        (or -> missing if a grasp failed)
Box status:   empty -> packing -> packed               (location 'gripper' while being carried)
"""

from __future__ import annotations

import sqlite3
from dataclasses import dataclass, fields
from datetime import datetime
from pathlib import Path

GRIPPER = "gripper"
SHIPPED = "shipped"  # boxes taken away from the output spots

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
CREATE TABLE IF NOT EXISTS boxes (
    id TEXT PRIMARY KEY,
    location TEXT NOT NULL,
    status TEXT NOT NULL CHECK (status IN ('empty','packing','packed')),
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
    """A part row; `location` is a slot name, a box id once packed, or "gripper"."""

    id: int
    sku: str
    location: str
    status: str
    box_id: str | None
    box_slot: int | None


@dataclass
class Box:
    """A box row; `location` is a spot name, "gripper" or "shipped"."""

    id: str
    location: str
    status: str


def _part(row: sqlite3.Row) -> Part:
    return Part(**{f.name: row[f.name] for f in fields(Part)})


def _box(row: sqlite3.Row) -> Box:
    return Box(row["id"], row["location"], row["status"])


def _now() -> str:
    return datetime.now().isoformat(timespec="seconds")


class InventoryError(RuntimeError):
    """An inventory change that doesn't make sense (unknown id, wrong status, occupied spot)."""


# Event row: (kind, part_id, box_id, detail)
Event = tuple[str, int | None, str | None, str]
Statement = tuple[str, tuple]


class Inventory:
    """SQLite-backed record of parts, boxes and an event log. Each change is one transaction."""

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

    def box(self, box_id: str) -> Box:
        row = self.db.execute("SELECT * FROM boxes WHERE id = ?", (box_id,)).fetchone()
        if not row:
            raise InventoryError(f"no box {box_id}")
        return _box(row)

    def parts(self, status: str | None = None) -> list[Part]:
        if status:
            rows = self.db.execute("SELECT * FROM parts WHERE status = ? ORDER BY id", (status,))
        else:
            rows = self.db.execute("SELECT * FROM parts ORDER BY id")
        return [_part(row) for row in rows]

    def boxes(self, status: str | None = None) -> list[Box]:
        if status:
            rows = self.db.execute("SELECT * FROM boxes WHERE status = ? ORDER BY id", (status,))
        else:
            rows = self.db.execute("SELECT * FROM boxes ORDER BY id")
        return [_box(row) for row in rows]

    def box_at(self, location: str) -> Box | None:
        row = self.db.execute("SELECT * FROM boxes WHERE location = ?", (location,)).fetchone()
        return _box(row) if row else None

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

    def contents(self, box_id: str) -> dict[int, Part]:
        """Packed parts in a box, by box slot."""
        rows = self.db.execute("SELECT * FROM parts WHERE box_id = ? AND status = 'packed'", (box_id,))
        return {row["box_slot"]: _part(row) for row in rows}

    def in_gripper(self) -> list[str]:
        """Human-readable list of everything the inventory says the gripper is holding."""
        held = [f"part {p.id} ({p.sku})" for p in self.parts("in_gripper")]
        held += [f"box {b.id}" for b in self.boxes() if b.location == GRIPPER]
        return held

    def events(self, limit: int = 20) -> list[sqlite3.Row]:
        """Most recent events, newest first."""
        return list(self.db.execute("SELECT * FROM events ORDER BY id DESC LIMIT ?", (limit,)))

    # ---------- stocking ----------
    def restock(self, part_slots: dict[str, str], box_supply: list[str]) -> tuple[int, int]:
        """Put one available part in every empty part slot and one empty box on every free supply spot.
        Call after physically refilling the slots. Returns (parts added, boxes added)."""
        added_parts = added_boxes = 0
        for location, sku in part_slots.items():
            if not self.available_part(location):
                self._tx(
                    [("INSERT INTO parts (sku, location, status, updated_at) VALUES (?, ?, 'available', ?)",
                      (sku, location, _now()))],
                    ("restock_part", None, None, f"{sku} @ {location}"),
                )
                added_parts += 1
        box_count = self.db.execute("SELECT COUNT(*) FROM boxes").fetchone()[0]
        for location in box_supply:
            if not self.box_at(location):
                box_count += 1
                box_id = f"BOX-{box_count:04d}"
                self._tx(
                    [("INSERT INTO boxes (id, location, status, updated_at) VALUES (?, ?, 'empty', ?)",
                      (box_id, location, _now()))],
                    ("restock_box", None, box_id, location),
                )
                added_boxes += 1
        return added_parts, added_boxes

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

    def pack_part(self, part_id: int, box_id: str, slot: int) -> None:
        """in_gripper -> packed into `slot` of `box_id`."""
        if self.part(part_id).status != "in_gripper":
            raise InventoryError(f"part {part_id} is not in the gripper")
        self._tx(
            [("""
              UPDATE parts
              SET status = 'packed', location = ?, box_id = ?, box_slot = ?, updated_at = ?
              WHERE id = ?
              """,
              (box_id, box_id, slot, _now(), part_id))],
            ("pack_part", part_id, box_id, f"slot {slot}"),
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

    def move_box(self, box_id: str, location: str, status: str | None = None) -> None:
        """Record that a box is now at `location` (and optionally has a new status)."""
        box = self.box(box_id)
        if location not in (GRIPPER, SHIPPED):
            other = self.box_at(location)
            if other and other.id != box_id:
                raise InventoryError(f"{location} already has box {other.id}")
        new_status = status or box.status
        self._tx(
            [("UPDATE boxes SET location = ?, status = ?, updated_at = ? WHERE id = ?",
              (location, new_status, _now(), box_id))],
            ("move_box", None, box_id, f"{box.location}->{location} {new_status}"),
        )

    # ---------- reporting ----------
    def summary(self) -> str:
        """Multi-line status report: part counts, box locations, anything stuck in the gripper."""
        lines = ["PARTS"]
        counts = self.db.execute(
            "SELECT sku, status, COUNT(*) n FROM parts GROUP BY sku, status ORDER BY sku, status"
        )
        for row in counts:
            lines.append(f"  {row['sku']:<12} {row['status']:<11} {row['n']}")
        lines.append("BOXES")
        boxes = self.boxes()
        shipped = [b for b in boxes if b.location == SHIPPED]
        if shipped:
            lines.append(f"  shipped: {len(shipped)} boxes")
        for box in boxes:
            if box.location == SHIPPED:
                continue
            lines.append(f"  {box.id}  {box.status:<8} @ {box.location:<14} parts: {len(self.contents(box.id))}")
        held = self.in_gripper()
        if held:
            lines.append("!! IN GRIPPER (resolve before running): " + ", ".join(held))
        return "\n".join(lines)
