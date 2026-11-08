"""Cell procedures: pick/place primitives and the resumable packaging job.

Every location L is two taught poses: `L.above` (safe approach height) and `L.at` (grasp/release).
Between stations the arm goes through `travel_pose` so long moves never sweep at table height.
"""

from __future__ import annotations

from typing import Callable

from ..config import CellConfig
from ..motion.controller import GraspError, Motion
from .inventory import GRIPPER, Box, Inventory, Part


class CellError(RuntimeError):
    """A condition the operator must fix (out of stock, output full, item in gripper...)."""


class Cell:
    """Runs pick/place procedures on a `Motion`, recording every step in the `Inventory`."""

    def __init__(self, cfg: CellConfig, motion: Motion, inventory: Inventory,
                 log: Callable[[str], None] = print):
        self.cfg, self.motion, self.inventory, self.log = cfg, motion, inventory, log

    # ---------- primitives ----------
    def travel(self) -> None:
        """Go to the safe travel pose (if configured)."""
        if self.cfg.travel_pose:
            self.motion.move_to(self.cfg.travel_pose)

    def pick(self, loc: str, min_width: float) -> float:
        """Pick whatever is at `loc`; returns the grip width. Raises GraspError (gripper reopened,
        arm back above) if the grip is narrower than `min_width`."""
        motion, slow = self.motion, self.cfg.motion.approach_speed
        motion.move_to(f"{loc}.above")
        motion.open_gripper()
        motion.move_to(f"{loc}.at", speed=slow)
        try:
            width = motion.grip(min_width)
        except GraspError:
            motion.open_gripper()
            motion.move_to(f"{loc}.above", speed=slow)
            raise
        motion.move_to(f"{loc}.above", speed=slow)
        return width

    def place(self, loc: str) -> None:
        """Put the held item down at `loc` and retreat upwards."""
        motion, slow = self.motion, self.cfg.motion.approach_speed
        motion.move_to(f"{loc}.above")
        motion.move_to(f"{loc}.at", speed=slow)
        motion.open_gripper()
        motion.move_to(f"{loc}.above", speed=slow)

    # ---------- operations ----------
    def transfer_box(self, box_id: str, dst: str, status: str | None = None) -> None:
        """Carry a box from where the inventory says it is to `dst`, optionally setting its status."""
        box = self.inventory.box(box_id)
        other = self.inventory.box_at(dst)
        if other and other.id != box_id:
            raise CellError(f"{dst} is occupied by {other.id}")
        src = box.location
        self.log(f"box {box_id}: {src} -> {dst}")
        self.travel()
        self.inventory.move_box(box_id, GRIPPER)  # committed before the grasp: a crash leaves it flagged
        try:
            self.pick(src, self.cfg.gripper.min_box_width)
        except GraspError:
            self.inventory.move_box(box_id, src)
            raise
        self.travel()
        self.place(dst)
        self.inventory.move_box(box_id, dst, status)

    def pack_part(self, part: Part, box_id: str, slot: int) -> None:
        """Move one part into a box slot. A missed grasp marks the part `missing` and re-raises."""
        self.log(f"part {part.id} ({part.sku}) {part.location} -> {box_id} slot {slot}")
        self.travel()
        self.inventory.pick_part(part.id)
        try:
            self.pick(part.location, self.cfg.gripper.min_part_width)
        except GraspError as e:
            self.inventory.set_part(part.id, "missing")
            self.log(f"  missed part {part.id}: {e}. Marked missing.")
            raise
        self.travel()
        self.place(self.cfg.box_slots[slot % len(self.cfg.box_slots)].pose)
        self.inventory.pack_part(part.id, box_id, slot)

    def _candidates(self, sku: str | None) -> list[Part]:
        """Available parts (one per slot) matching `sku`; None matches any."""
        candidates = []
        for location, slot_sku in self.cfg.part_slots.items():
            if sku is None or slot_sku == sku:
                part = self.inventory.available_part(location)
                if part:
                    candidates.append(part)
        return candidates

    def check_ready(self) -> None:
        """Refuse to run while the inventory says something is in the gripper."""
        held = self.inventory.in_gripper()
        if held:
            raise CellError("inventory says the gripper holds " + ", ".join(held) +
                            ". Remove it by hand and fix with `set-part` / `set-box` first.")

    def package_box(self, skus: list[str] | None = None) -> str:
        """Get a box to the pack station (or resume the one there), fill it, move it out.
        `skus`: what to put in the box (an order's items, up to `max_items_per_order`); item i is box slot i,
        dropped at `box_slots[i % len(box_slots)]`. None = the fixed recipe from `box_slots`. Safe to call
        again after any stop: it resumes from the inventory."""
        if not self.cfg.box_slots:
            raise CellError("no box_slots (drop spots) configured")
        if skus is not None and len(skus) > self.cfg.max_items_per_order:
            raise CellError(f"{len(skus)} items but an order holds at most {self.cfg.max_items_per_order}")
        if skus is not None:
            plan = list(enumerate(skus))
        else:
            plan = [(slot, box_slot.sku) for slot, box_slot in enumerate(self.cfg.box_slots)]
        self.check_ready()
        box = self._box_at_station()
        self._fill_box(box, plan)
        self.inventory.move_box(box.id, self.cfg.pack_station, "packed")
        self._ship(box.id)
        self.travel()
        return box.id

    def _box_at_station(self) -> Box:
        """The box to work on: resume the one at the pack station, or bring a new empty one.
        A finished box left at the station is shipped first."""
        station = self.cfg.pack_station
        box = self.inventory.box_at(station)
        if box and box.status == "packed":
            self._ship(box.id)
            box = None
        if box is None:
            return self._bring_empty_box(station)
        if box.status == "empty":
            self.inventory.move_box(box.id, station, "packing")
        else:
            self.log(f"resuming box {box.id} at {station}")
        return box

    def _bring_empty_box(self, station: str) -> Box:
        empties = [b for b in self.inventory.boxes("empty") if b.location in self.cfg.box_supply]
        if not empties:
            raise CellError("no empty boxes in supply — restock boxes, then run `restock`")
        box = empties[0]
        self.transfer_box(box.id, station, "packing")
        return box

    def _fill_box(self, box: Box, plan: list[tuple[int, str | None]]) -> None:
        """Pack every planned (slot, sku) not already in the box, retrying with another part after a miss."""
        filled = self.inventory.contents(box.id)
        for slot, sku in plan:
            if slot in filled:
                continue
            while True:
                candidates = self._candidates(sku)
                if not candidates:
                    self.travel()
                    raise CellError(
                        f"out of stock for slot {slot} (sku={sku or 'any'}); box {box.id} "
                        f"stays at {self.cfg.pack_station}. Refill, run `restock`, then run again to resume.")
                try:
                    self.pack_part(candidates[0], box.id, slot)
                    break
                except GraspError:
                    continue  # that part was marked missing; try the next one

    def _ship(self, box_id: str) -> None:
        """Move a packed box to the first free output spot."""
        free = [loc for loc in self.cfg.box_output if not self.inventory.box_at(loc)]
        if not free:
            raise CellError(f"all output spots are full; box {box_id} stays at the pack station. "
                            "Clear output and use `set-box` to remove the shipped boxes.")
        self.transfer_box(box_id, free[0])
