"""Pack an order on the arm owned by the server's ArmHub: the box to the pack spot, each reserved part from the
tray into it, the box to its output spot (packing/boxes/pack.py). Used by the robot agent (`start --robot`).

Demo quality (ROADMAP "Next steps"): moves are assumed to succeed; no resume, no retries.
- Before anything moves: no free box / another box still at the pack spot / unplannable -> `NotStarted` (the box
  isn't kept; the robot agent puts the order back to "received" and keeps going).
- Each part is marked used as soon as it's out of its pocket and held (tray map + stock update live).
- The gripper closes on nothing (or the part drops while pulling it out) -> that pocket is marked empty, the order
  stops (`PocketEmpty`). Whatever already happened (box at the pack spot, parts in it) stays; a person sorts it out.
- Done: the box is `packed` at `output` (its supply spot mirrored right) until someone clicks "Picked up".
"""

from __future__ import annotations

import logging
import time
from typing import Callable

from ..motion.controller import GraspError, Motion
from ..motion.program import JointMove, run
from ..poses import PoseStore
from ..service.orders import Order
from .geometry import obstacles
from .move import box_pose
from .pack import plan_pack_order
from .store import BoxError, BoxStore

log = logging.getLogger(__name__)


class NotStarted(Exception):
    """Refused before the arm moved (message for the user). The order can simply be actioned again."""
    not_started = True


class PocketEmpty(Exception):
    """The gripper closed on nothing at this pocket: it's marked empty and the order stops."""

    def __init__(self, pocket: str, msg: str):
        super().__init__(msg)
        self.pocket = pocket


class TrayPacker:
    def __init__(self, hub, cfg, boxes: BoxStore, shelf, record: Callable[..., None] = lambda *a, **k: None,
                 sleep: Callable[[float], None] = time.sleep):
        self.hub, self.cfg, self.boxes, self.shelf, self.record, self.sleep = hub, cfg, boxes, shelf, record, sleep

    def check_pack_spot(self, order: Order) -> None:
        """BoxError if another box is still at the pack spot."""
        for b in self.boxes.list():
            if b.location == "pack" and b.order_id != order.id:
                raise BoxError(f"box {b.id} is still at the pack spot — move it to the right or remove it first")

    def plan(self, order: Order, current: dict[str, float] | None = None, dry_run: bool = False):
        """Plan (no motion). Takes / reuses the order's box (dry_run: only looks which one it would be).
        Raises on anything unplannable."""
        self.check_pack_spot(order)
        box = self.boxes.peek(order.id) if dry_run else self.boxes.take(order.id)
        model = self.hub.model
        if model is None:
            raise RuntimeError("kinematics unavailable (placo not installed?)")
        current = current or self.hub.state()["joints"]
        if not current:
            raise RuntimeError("no arm reading yet")
        poses = {b.id: box_pose(b, b.location, self.cfg, model.pan_xy) for b in self.boxes.list()}
        with self.hub._lock:
            return plan_pack_order(model, self.cfg, self.cfg.tray.calibration(), order, box, current,
                                   obstacles(self.cfg, poses, skip=box.id))

    def pack(self, order: Order) -> str:
        with self.hub.borrow(mode="packing") as arm:
            if not arm.torque:
                arm.write(self.hub.hold_target(arm.read()))
                arm.set_torque(True)
            had_box = self._box_of(order)
            try:
                plan = self.plan(order, {j: v for j, v in arm.read().items() if j != "gripper"})
            except Exception as e:  # nothing has moved: give the box back, the order can be actioned again
                if had_box is None and (b := self._box_of(order)) is not None:
                    self.boxes.update(b.id, state="available", order_id=None)
                raise NotStarted(str(e)) from e
            n = len(plan.items)
            self.record("robot", f"Packing order {order.id}: {n} item(s) into box {plan.box_id}, "
                                 f"{len(plan.steps)} steps", order_id=order.id, box_id=plan.box_id)
            motion = Motion(arm, PoseStore(self.cfg.poses_file), self.cfg.motion, self.cfg.gripper,
                            sleep=self.sleep, log=log.info, gravity=getattr(self.hub, "gravity", None))
            slow = self.cfg.motion.speed / 2
            steps = [JointMove(s.pose, s.speed or slow, s.label) if isinstance(s, JointMove) else s for s in plan.steps]
            # step index -> what happens when that step STARTS (on_step runs before each step)
            index = {id(st): k for k, st in enumerate(steps)}
            starts = {r["start"]: i for i, r in enumerate(plan.item_steps)}
            taken = {r["taken"]: i for i, r in enumerate(plan.item_steps)}
            dropped = {r["end"]: i for i, r in enumerate(plan.item_steps)}
            # the box is down at the pack spot once the step after its first release starts
            state = {"at_pack": self.boxes.get(plan.box_id).location == "pack", "released": False,
                     "item": None}  # item being picked (not yet out of its pocket)

            def on_step(st):
                log.info("order %s: %s", order.id, st.label)
                k = index.get(id(st))  # None for recovery (on_fail) steps
                if state["released"] and not state["at_pack"]:
                    self.boxes.update(plan.box_id, location="pack")
                    state["at_pack"] = True
                if st.label == "release the box" and not state["at_pack"]:
                    state["released"] = True
                if k in dropped:
                    it = plan.items[dropped[k]]
                    self.record("robot", f"Order {order.id} item {dropped[k] + 1}/{n}: {it['sku']} from "
                                         f"{it['pocket']} → box {plan.box_id}", order_id=order.id, pocket=it["pocket"])
                if k in taken:
                    slot = plan.items[taken[k]]["pocket"]
                    self.shelf.consume_slots([slot])
                    state["item"] = None
                if k in starts:
                    state["item"] = starts[k]
            try:
                run(steps, motion, on_step=on_step)
            except GraspError as e:
                if state["item"] is None:
                    raise
                slot = plan.items[state["item"]]["pocket"]
                self.shelf.set_slot(slot, False)
                self.boxes.orders.release_slot(slot, self.shelf.stock())  # its reservation moves or goes short
                self.record("error", f"Order {order.id}: nothing to grip at {slot} — marked it empty; order "
                                     f"stopped", order_id=order.id, pocket=slot)
                raise PocketEmpty(slot, f"nothing to grip at {slot} (marked empty): {e}") from e
        self.boxes.update(plan.box_id, location="output", state="packed")
        self.record("robot", f"Order {order.id} packed into box {plan.box_id} (now on the right)",
                    order_id=order.id, box_id=plan.box_id)
        return f"BOX-{plan.box_id}"

    def _box_of(self, order: Order):
        return next((b for b in self.boxes.list() if b.order_id == order.id and b.state == "in_use"), None)
