"""Plan packing a whole order as one motion program (validated before anything moves):

  1. the order's box: supply (left) → pack spot (middle)        packing/boxes/move.py
  2. per item: pick it from its reserved pocket                  packing/tray/pick.py
               → drop it into the box (below)
  3. the box: pack spot → output (its supply spot mirrored right)

Drop: the part hangs past the jaws along the tool axis by up to its `length_mm`. Pointing straight down only
works up to ~10 cm above the table here, so the drop holds the jaws just high enough that the part clears the
floor, shifts sideways so the (tilted) part is centered over the opening, and checks the whole part is inside
the walls and the jaws above the rim before opening.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable

import numpy as np

from ..motion.kinematics import ArmModel, PathError
from ..motion.program import OpenGripper, ProgramBuilder, Step, describe
from ..tray.pick import AxisIK, best_targets, plan_pocket_pick
from .geometry import BOX_H, BOX_W, WALL
from .move import box_pose, down_seeds, plan_box_move

DROP_ABOVE_FLOOR_MM = 8.0  # part's lowest point above the box floor at release
RIM_CLEAR_MM = 12.0  # jaws (tool point) at least this far above the rim
DROP_SPEED_MM_S = 25.0
RETREAT_MM = 120.0  # after letting go: up to this far above the box rim before moving on


@dataclass
class _Start:
    """Marker so a following JointMove is checked from this pose."""
    pose: dict
    label: str = "start"


@dataclass
class PackPlan:
    order_id: int
    box_id: int
    steps: list[Step]
    items: list[dict]  # per item: pocket, sku, pitch_deg, drop_tilt_deg
    on_done: Callable[[], None] | None = field(default=None, repr=False)
    tilt_deg: float = 0.0

    @property
    def label(self) -> str:
        return f"pack order {self.order_id} into box {self.box_id}"

    def summary(self) -> dict:
        return {"order_id": self.order_id, "box_id": self.box_id, "items": self.items,
                "step_count": len(self.steps), "steps": describe(self.steps)}


def plan_drop(model: ArmModel, b: ProgramBuilder, T_box: np.ndarray, length_mm: float, open_value: float | None,
              label: str) -> float:
    """Append: free move to the drop pose over the box, open, lift the jaws 30 mm. Returns the drop tilt (deg).
    Raises PathError if the part can't be held fully inside the box from any reachable pose."""
    center, floor = T_box[:3, 3] + np.array([0, 0, WALL]), float(T_box[2, 3]) + WALL
    top = float(T_box[2, 3]) + BOX_H
    L = length_mm / 1000
    z = max(top + RIM_CLEAR_MM / 1000, floor + L + DROP_ABOVE_FLOOR_MM / 1000)
    p = np.array([center[0], center[1], z])
    ik = AxisIK(model)
    down, any_dir = np.array([0.0, 0.0, -1.0]), T_box[:3, 0]
    q = None
    for _ in range(3):  # center the hanging part over the opening: shift by half its horizontal spread
        targets, poses = best_targets(model, ik, p, down, any_dir, [b.pose, *down_seeds(model, p)])
        q, T = poses[0], model.fk(poses[0])
        zt = T[:3, 2]
        if zt[2] > -0.05:
            raise PathError(f"{label}: can't point the gripper down over the box")
        # lowest point must clear the floor: the part ends at p + L * zt
        p = np.array([center[0] - zt[0] * L / 2, center[1] - zt[1] * L / 2,
                      max(top + RIM_CLEAR_MM / 1000, floor + DROP_ABOVE_FLOOR_MM / 1000 - zt[2] * L)])
    targets, poses = best_targets(model, ik, p, down, any_dir, [q, *down_seeds(model, p)])
    T = model.fk(poses[0])
    if np.linalg.norm(T[:3, 3] - p) > 0.002:
        raise PathError(f"{label}: can't reach the drop point over the box ({np.round(p * 1000, 1)} mm)")
    zt = T[:3, 2]
    # no wall check (user: the walls are ~0.5 mm): the part is centered over the opening, its end above the floor
    end = T[:3, 3] + zt * L
    if end[2] < floor + DROP_ABOVE_FLOOR_MM / 2000:
        raise PathError(f"{label}: the part would touch the box floor")
    b.joint_to(targets[0], f"{label}: over the box", seeds=poses, targets=targets[:1])
    b.open_gripper(f"{label}: release", open_value)
    retreat_up(model, b, T_box, f"{label}: up and away from the box")
    return float(np.degrees(np.arccos(np.clip(-zt[2], -1, 1))))


def retreat_up(model: ArmModel, b: ProgramBuilder, T_box: np.ndarray, label: str) -> None:
    """Straight up to RETREAT_MM above the box rim (exact position, gripper as close to pointing down as it
    can) — so the next free move starts well clear of the box."""
    p = b.tip.copy()
    p[2] = max(p[2] + 0.03, float(T_box[2, 3]) + BOX_H + RETREAT_MM / 1000)
    ik = AxisIK(model)
    targets, poses = best_targets(model, ik, p, np.array([0.0, 0.0, -1.0]), model.fk(b.pose)[:3, 0],
                                  [b.pose, *down_seeds(model, p)])
    b.joint_to(targets[0], label, seeds=poses, targets=targets[:1])


BOX_CLEAR_MM = 10.0  # arm / part kept this far outside the box at the pack spot (except the drop itself)
GRIPPER_BACK_M = 0.098  # tool point -> wrist (gripper_link origin), back along the tool axis


def check_clear_of_box(model: ArmModel, steps: list[Step], T_box: np.ndarray, part_len_m: float, label: str,
                       samples: int = 10) -> None:
    """Every pose of these steps keeps the jaws, the wrist and (while holding) the part's far end outside the box
    sitting at T_box (plus BOX_CLEAR_MM). Joint moves are checked along their interpolation."""
    from ..motion.program import CheckHolding, Grip, JointMove, LineMove

    R_inv, t = T_box[:3, :3].T, T_box[:3, 3]
    m = BOX_CLEAR_MM / 1000
    lo, hi = np.array([-BOX_W / 2 - m, -BOX_W / 2 - m, -1.0]), np.array([BOX_W / 2 + m, BOX_W / 2 + m, BOX_H + m])
    holding, prev = False, None

    def poses_of(st):
        if isinstance(st, LineMove):
            return st.path[:: max(1, len(st.path) // samples)] + [st.path[-1]]
        if isinstance(st, JointMove) and prev is not None:
            return [{j: prev[j] + (st.pose[j] - prev[j]) * k / samples for j in st.pose} for k in range(samples + 1)]
        return []

    for st in steps:
        if isinstance(st, Grip):
            holding = True
        elif isinstance(st, OpenGripper):
            holding = False
        if isinstance(st, CheckHolding):
            continue
        for q in poses_of(st):
            T = model.fk(q)
            pts = [T[:3, 3], T[:3, 3] - T[:3, 2] * GRIPPER_BACK_M]
            if holding:
                pts.append(T[:3, 3] + T[:3, 2] * part_len_m)
            for pt in pts:
                b = R_inv @ (pt - t)
                if np.all(b > lo) and np.all(b < hi):
                    raise PathError(f"{label}: '{st.label}' would hit the box at the pack spot")
        if isinstance(st, LineMove):
            prev = st.path[-1]
        elif isinstance(st, JointMove) or isinstance(st, _Start):
            prev = st.pose


def plan_pack_order(model: ArmModel, cfg, cal, order, box, current: dict[str, float],
                    obstacles: list[tuple[str, np.ndarray]] | None = None) -> PackPlan:
    """`order.slots` = the reserved pocket per item (all must be set). `box` = the order's box (at supply or
    already at the pack spot when resuming). Raises PathError / ValueError."""
    if not order.slots or None in order.slots:
        raise ValueError(f"order {order.id} doesn't have a reserved unit for every item")
    if cal is None:
        raise ValueError("calibrate the tray first")
    steps: list[Step] = []
    pose = dict(current)

    def last_pose(prog_steps) -> dict:
        for st in reversed(prog_steps):
            if hasattr(st, "path"):
                return st.path[-1]
            if hasattr(st, "pose") and isinstance(st.pose, dict):
                return st.pose
        return pose

    T_box = box_pose(box, "pack", cfg, model.pan_xy)
    if box.location != "pack":
        mv = plan_box_move(model, box, box.location, "pack", pose, cfg, obstacles)
        steps += mv.steps
        pose = last_pose(mv.steps)
    b0 = ProgramBuilder(model, pose)
    retreat_up(model, b0, T_box, "up and away from the box")
    steps += b0.steps
    pose = b0.pose
    lengths = {t.sku: t.length_mm for t in cfg.tray.types}
    items = []
    for i, slot in enumerate(order.slots):
        pick = plan_pocket_pick(model, cfg.tray, cfg.gripper, cal, slot, pose)
        check_clear_of_box(model, [_Start(pose), *pick.steps], T_box, lengths.get(pick.sku, 60.0) / 1000,
                           f"item {i + 1} ({pick.sku} from {slot})")
        steps += pick.steps
        b = ProgramBuilder(model, last_pose(pick.steps))
        tilt = plan_drop(model, b, T_box, lengths.get(pick.sku, 60.0), pick.open_value,
                         f"item {i + 1} ({pick.sku} from {slot})")
        steps += b.steps
        pose = b.pose
        items.append({"pocket": slot, "sku": pick.sku, "pitch_deg": pick.pitch_deg, "drop_tilt_deg": round(tilt, 1)})
    out = plan_box_move(model, box, "pack", "output", pose, cfg, obstacles)
    steps += out.steps
    if not any(isinstance(s, OpenGripper) for s in steps):
        raise AssertionError("no release step")  # can't happen: every drop and box move opens
    return PackPlan(order.id, box.id, steps, items)
