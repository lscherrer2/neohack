"""Real stock, from the packing cell's inventory database (packing/data/inventory.db).

Opens the database per call: SQLite connections belong to the thread that made them, and the server,
the dashboard and the robot agent thread all read stock.
"""

from __future__ import annotations

import time

from ..cell.inventory import Inventory
from ..service.orders import Order


class Shelf:
    def __init__(self, cfg):
        self.cfg = cfg
        self.path, self.box_supply = cfg.inventory_db, cfg.box_supply

    @property
    def slots(self) -> dict[str, str]:
        """location -> sku for every part slot, tray pockets included."""
        return self.cfg.stock_slots

    def _inv(self) -> Inventory:
        return Inventory(self.path)

    def available(self) -> dict[str, int]:
        out: dict[str, int] = {}
        for p in self._inv().parts("available"):
            out[p.sku] = out.get(p.sku, 0) + 1
        return out

    def restock(self) -> tuple[int, int]:
        """After refilling by hand: one part in every empty part slot, one box on every empty supply spot."""
        return self._inv().restock(self.slots, self.box_supply)

    def full_slots(self) -> set[str]:
        return self._inv().full_slots() & set(self.slots)

    def set_slot(self, location: str, full: bool) -> bool:
        """A human marks one slot full/empty. KeyError for an unknown slot."""
        return self._inv().set_slot(location, self.slots[location], full)

    def set_all(self, full: bool, sku: str | None = None, only: set[str] | None = None) -> int:
        """Mark every slot (of one sku; within `only` if given) full or empty. Returns how many changed."""
        inv = self._inv()
        return sum(inv.set_slot(loc, s, full) for loc, s in self.slots.items()
                   if sku in (None, s) and (only is None or loc in only))

    def consume(self, skus: list[str]) -> None:
        """Mark one available part per SKU as used (simulated packing; the real cell does this itself)."""
        inv = self._inv()
        for sku in skus:
            part = next((p for p in inv.parts("available") if p.sku == sku), None)
            if part is None:
                raise RuntimeError(f"no {sku} left on the shelf")
            inv.set_part(part.id, "packed")


class ShelfSimPacker:
    """Simulated packing that still uses up real inventory, so stock numbers stay true."""

    def __init__(self, shelf: Shelf, seconds_per_item: float = 3.0):
        self.shelf, self.seconds_per_item = shelf, seconds_per_item

    def pack(self, order: Order) -> str:
        time.sleep(self.seconds_per_item * len(order.items))
        self.shelf.consume(order.items)
        return f"SIM-BOX-{order.id}"
