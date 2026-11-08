"""Pack an order on the arm owned by the server's ArmHub: the box to the pack spot, each reserved part from the
tray into it, the box to its output spot (packing/boxes/pack.py). Used by the robot agent (`start --robot`).

Everything is planned and validated before the arm moves. On success: the reserved units are marked used, the
box is `packed` at `output`. On failure the exception goes back to the robot agent (order -> failed, agent
stops); where the box ended up is unknown — a person checks it (dashboard: Reset / Remove box).
"""

from __future__ import annotations

import logging
import time
from typing import Callable

from ..motion.controller import Motion
from ..motion.program import JointMove, run
from ..poses import PoseStore
from ..service.orders import Order
from .geometry import obstacles
from .move import box_pose
from .pack import plan_pack_order
from .store import BoxStore

log = logging.getLogger(__name__)


class TrayPacker:
    def __init__(self, hub, cfg, boxes: BoxStore, shelf, record: Callable[..., None] = lambda *a, **k: None,
                 sleep: Callable[[float], None] = time.sleep):
        self.hub, self.cfg, self.boxes, self.shelf, self.record, self.sleep = hub, cfg, boxes, shelf, record, sleep

    def plan(self, order: Order, current: dict[str, float] | None = None, dry_run: bool = False):
        """Plan (no motion). Takes / reuses the order's box (dry_run: only looks which one it would be).
        Raises on anything unplannable."""
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
            plan = self.plan(order, {j: v for j, v in arm.read().items() if j != "gripper"})
            self.record("robot", f"Packing order {order.id}: {len(plan.items)} item(s) into box {plan.box_id}, "
                                 f"{len(plan.steps)} steps", order_id=order.id, box_id=plan.box_id)
            motion = Motion(arm, PoseStore(self.cfg.poses_file), self.cfg.motion, self.cfg.gripper,
                            sleep=self.sleep, log=log.info, gravity=getattr(self.hub, "gravity", None))
            slow = self.cfg.motion.speed / 2
            steps = [JointMove(s.pose, s.speed or slow, s.label) if isinstance(s, JointMove) else s for s in plan.steps]
            # on_step runs BEFORE each step: the box is down at the pack spot once the step after its first
            # release starts (a retry then plans from there). Resuming (already at pack): nothing to record.
            state = {"at_pack": self.boxes.get(plan.box_id).location == "pack", "released": False}

            def on_step(st):
                log.info("order %s: %s", order.id, st.label)
                if state["released"] and not state["at_pack"]:
                    self.boxes.update(plan.box_id, location="pack")
                    state["at_pack"] = True
                if st.label == "release the box" and not state["at_pack"]:
                    state["released"] = True
            run(steps, motion, on_step=on_step)
        self.shelf.consume_slots(list(order.slots))
        self.boxes.update(plan.box_id, location="output", state="packed")
        self.record("robot", f"Order {order.id} packed into box {plan.box_id} (now on the right)",
                    order_id=order.id, box_id=plan.box_id)
        return f"BOX-{plan.box_id}"
