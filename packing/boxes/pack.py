"""Plan packing a whole order as one motion program (validated before anything moves):

  1. the order's box: supply (left) → pack spot (middle)        packing/boxes/move.py
  2. per item: pick it from its reserved pocket                  packing/tray/pick.py
               → drop it into the box (below)
  3. the box: pack spot → output (its supply spot mirrored right)

Release (side release): the part sticks out past the jaws along the tool axis by its `length_mm`. Pointing the
gripper down only reached ~10 cm above the table, so instead the wrist rolls (during the free move from the pick)
until the jaws close HORIZONTALLY (tool x level): when they open nothing is under the part and it falls straight
down. The tool points away from the swivel over the box, level or pitched down (level first: most reach; a part
longer than the opening is first tried pitched just enough to fit inside it, so it drops in end first). The part's
center (tool point + length/2 along the tool axis, beside the fixed jaw) is over the box center; jaws and the
whole part RIM_CLEAR_MM above the rim. The way in, the release and the retreat are checked clear of the box.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable

import numpy as np

from ..motion.kinematics import POSITION_TOL_M, ArmModel, PathError
from ..motion.program import OpenGripper, ProgramBuilder, Step, describe
from ..tray.pick import AxisIK, best_targets, pitched, plan_pocket_pick
from .geometry import BOX_H, BOX_W, WALL
from .move import box_pose, down_seeds, plan_box_move, rest_move

RIM_CLEAR_MM = 12.0  # jaws (tool point) and the part's underside at least this far above the rim at the release
DROP_PITCHES_DEG = (0.0, 10.0, 20.0, 30.0, 45.0, 60.0)  # release tool pitch below level, tried in order
FIT_MARGIN_MM = 4.0  # a part "fits the opening" when its span is this far inside each wall (≥ the 4 mm centering tol)
MAX_JAW_TILT_DEG = 10.0  # jaw closing axis (tool x) at most this far off level at the release
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
    items: list[dict]  # per item: pocket, sku, pitch_deg, drop_tilt_deg (tool below level), drop_roll(_change)
    on_done: Callable[[], None] | None = field(default=None, repr=False)
    tilt_deg: float = 0.0
    # per item, indices into `steps`: "start" = first step of its pick, "taken" = first step after the pick
    # (the part is out of the pocket and held), "end" = one past its drop
    item_steps: list[dict] = field(default_factory=list)

    @property
    def label(self) -> str:
        return f"pack order {self.order_id} into box {self.box_id}"

    def summary(self) -> dict:
        return {"order_id": self.order_id, "box_id": self.box_id, "items": self.items,
                "step_count": len(self.steps), "steps": describe(self.steps)}


@dataclass
class DropInfo:
    """What the release pose looks like (for the plan summary / tests)."""
    pitch_deg: float  # tool axis below level (0 = level)
    roll: float  # wrist_roll at the release (units)
    roll_change: float  # wrist_roll turned on the way from the pick (units)
    jaw_tilt_deg: float  # jaw closing axis (tool x) vs level
    center_off_mm: float  # part center vs the box center, horizontally
    part_low_m: float  # lowest point of the part (center line − half its height)
    fits_opening: bool  # the part's horizontal span fits inside the opening (else it lands across the rim)


def _inside_mm() -> float:
    return (BOX_W - 2 * WALL) * 1000 - 2 * FIT_MARGIN_MM


def span_mm(length_mm: float, height_mm: float, pitch_deg: float) -> float:
    """Horizontal span of a held part (along the tool axis) with the tool pitched `pitch_deg` down."""
    a = np.radians(pitch_deg)
    return length_mm * np.cos(a) + height_mm * np.sin(a)


def drop_pitches(length_mm: float, height_mm: float) -> tuple[float, ...]:
    """Release pitches to try, in order: DROP_PITCHES_DEG; for a part longer than the opening, first the smallest
    pitch (5° steps) at which it fits inside the opening, and 5° more."""
    if span_mm(length_mm, height_mm, 0.0) <= _inside_mm():
        return DROP_PITCHES_DEG
    fit = next((float(a) for a in range(5, 90, 5) if span_mm(length_mm, height_mm, a) <= _inside_mm()), None)
    return DROP_PITCHES_DEG if fit is None else (fit, fit + 5.0, *DROP_PITCHES_DEG)


def part_height_mm(t, grip_mm: float = 0.0) -> float:
    """A tray type's size across the jaw axis and its long axis (vertical at the release): circle = Ø, rect = the
    side the jaws don't close on, pin_pair = its `grip_mm` (estimate). Unknown type: `grip_mm`."""
    if t is None:
        return grip_mm
    if t.feature == "rect":
        return t.size[1] if t.grip_axis == "x" else t.size[0]
    return t.grip_width_mm


def _drop_targets(model: ArmModel, ik: AxisIK, b: ProgramBuilder, T_box: np.ndarray, L: float, half_w: float,
                  off: float, pitch_deg: float, sign: float):
    """IK for one release candidate: tool pitched `pitch_deg` down from level, pointing away from the swivel over
    the box; jaws (tool x) level, toward the fixed jaw = `sign` · tangential. The tool point is placed so the part
    center (tool point + L/2 along z − `off` along x, see tray/pick.py FIXED_JAW_FACE_MM) is over the box center
    and both the jaws and the part are RIM_CLEAR_MM above the rim. Iterated: with 5 DOF the reached axes differ a
    little from the wanted ones (the tool point sits off the roll axis)."""
    c = T_box[:3, 3]
    rim = float(T_box[2, 3]) + BOX_H + RIM_CLEAR_MM / 1000
    radial = np.array([*(c[:2] - model.pan_xy), 0.0])
    radial /= np.linalg.norm(radial)
    z_want = pitched(radial, pitch_deg)
    x_want = sign * np.cross([0.0, 0.0, 1.0], radial)
    z, x = z_want, x_want
    seeds = [b.pose, *down_seeds(model, c)]
    targets = poses = None
    for _ in range(3):
        p = c - z * L / 2 + x * off
        # jaws above the rim; the part's center line (from the jaws to its end) half its width above it
        p[2] = max(rim, rim + half_w - min(0.0, float(z[2])) * L)
        targets, poses = best_targets(model, ik, p, z_want, x_want, seeds)
        T = model.fk(poses[0])
        z, x = T[:3, 2], T[:3, 0]
        seeds = [poses[0], *seeds]
    return p, targets, poses


def plan_drop(model: ArmModel, b: ProgramBuilder, T_box: np.ndarray, length_mm: float, open_value: float | None,
              label: str, part_height_mm: float = 0.0, jaw_offset_mm: float = 0.0,
              pitches: tuple[float, ...] | None = None) -> DropInfo:
    """Append: free move to the release pose over the box (the wrist roll turns on the way), open, retreat up.
    Release pose: jaws closing HORIZONTALLY (tool x level), so nothing is under the part when they open; tool
    level or pitched down (the first of `pitches` that works; default `drop_pitches`: level first, but a part
    longer than the opening is first tried pitched just enough to fit inside it, so it drops in end first instead
    of landing across the rim); the part's center over the box center; jaws and part RIM_CLEAR_MM above the rim.
    Of the two jaw sides (wrist roll 180° apart) the one needing less roll from the pick wins.
    `part_height_mm` = the part's size across the jaw axis and its long axis (half of it hangs below the tool
    axis, see `part_height_mm()`); `jaw_offset_mm` = tool point → part center along the jaw axis
    (PickPlan.side_offset_mm). The way in, the release and the retreat are checked clear of the box (holding the
    part until the release). Raises PathError if no candidate works."""
    L, half_w, off = length_mm / 1000, part_height_mm / 2000, jaw_offset_mm / 1000
    if pitches is None:
        pitches = drop_pitches(length_mm, part_height_mm)
    ik = AxisIK(model)
    rim = float(T_box[2, 3]) + BOX_H + RIM_CLEAR_MM / 1000
    start_roll = b.pose["wrist_roll"]
    errors: list[PathError] = []
    for pitch in pitches:
        cands = []
        for sign in (1.0, -1.0):  # fixed jaw on either side (wrist roll 180° apart)
            try:
                p, targets, poses = _drop_targets(model, ik, b, T_box, L, half_w, off, pitch, sign)
            except PathError as e:
                errors.append(e)
                continue
            T = model.fk(poses[0])
            z, x = T[:3, 2], T[:3, 0]
            mid = T[:3, 3] + z * L / 2 - x * off
            low = T[2, 3] + min(0.0, float(z[2])) * L - half_w
            center_off = float(np.linalg.norm((mid - T_box[:3, 3])[:2]))
            jaw_tilt = float(np.degrees(np.arcsin(np.clip(abs(x[2]), 0, 1))))
            why = None
            if np.linalg.norm(T[:3, 3] - p) > POSITION_TOL_M:
                why = f"can't reach the release point over the box ({np.round(p * 1000, 1)} mm)"
            elif center_off > POSITION_TOL_M:
                why = f"part center {center_off * 1000:.0f} mm off the box center"
            elif jaw_tilt > MAX_JAW_TILT_DEG:
                why = f"jaws {jaw_tilt:.0f}° off level"
            elif low < rim - POSITION_TOL_M or T[2, 3] < rim - POSITION_TOL_M:
                why = "part / jaws not above the rim"
            if why:
                errors.append(PathError(f"{label}: pitch {pitch:.0f}°: {why}", p, list(b.steps), b.start))
                continue
            m = b.mark()
            try:
                b.joint_to(targets[0], f"{label}: over the box, jaws level", seeds=poses, targets=targets[:1])
                b.open_gripper(f"{label}: release", open_value)
                retreat_up(model, b, T_box, f"{label}: up and away from the box", keep_axis=True)
                check_clear_of_box(model, [_Start(m[1]), *b.steps[m[0]:]], T_box, L, label, holding=True)
            except PathError as e:
                errors.append(e)
                b.reset(m)
                continue
            q = b.steps[m[0]].pose
            info = DropInfo(float(np.degrees(np.arcsin(np.clip(-z[2], -1, 1)))), q["wrist_roll"], abs(q["wrist_roll"] - start_roll), jaw_tilt,
                            center_off * 1000, low,
                            bool(span_mm(length_mm, part_height_mm, pitch) <= _inside_mm()))
            cands.append((info.roll_change, info, b.steps[m[0]:], b.pose))
            b.reset(m)
        if cands:
            _, info, steps, end_pose = min(cands, key=lambda c: c[0])  # least wrist roll on the way
            b.steps.extend(steps)
            b.pose = end_pose
            return info
    last = errors[-1] if errors else PathError(f"{label}: no release pose")
    raise last.wrap(f"{label}: no release pose over the box (jaws level, tool pitch up to "
                    f"{max(pitches):.0f}°). Last: {last}", list(b.steps), b.start)


def retreat_up(model: ArmModel, b: ProgramBuilder, T_box: np.ndarray, label: str, keep_axis: bool = False) -> None:
    """Up and away from the box after letting go, so the next free move starts clear of it. Tries the highest
    reachable spot (RETREAT_MM … 40 mm above the rim), optionally a bit back toward the arm (always away from the
    box's far side), gripper pointing down if it can, else kept as it is (`keep_axis`: kept as it is first)."""
    tip, R = b.tip.copy(), model.fk(b.pose)[:3, :3]
    inward = np.array([*(model.pan_xy - tip[:2]), 0.0])
    inward /= max(np.linalg.norm(inward), 1e-9)
    rim = float(T_box[2, 3]) + BOX_H
    ik = AxisIK(model)
    axes = [np.array([0.0, 0.0, -1.0]), R[:, 2]]
    if keep_axis:
        axes.reverse()
    last = None
    for h in (RETREAT_MM, 90.0, 60.0, 40.0):
        for back in (0.0, 0.03, 0.06):
            p = tip + inward * back
            p[2] = max(tip[2] + 0.02, rim + h / 1000)
            for z_axis in axes:
                try:
                    targets, poses = best_targets(model, ik, p, z_axis, R[:, 0], [b.pose, *down_seeds(model, p)])
                    b.joint_to(targets[0], label, seeds=poses, targets=targets[:1])
                    return
                except PathError as e:
                    last = e
    raise (last or PathError("")).wrap(f"{label}: no reachable spot above the box ({last})", list(b.steps), b.start)


BOX_CLEAR_MM = 10.0  # arm / part kept this far outside the box at the pack spot (except the drop itself)
GRIPPER_BACK_M = 0.098  # tool point -> wrist (gripper_link origin), back along the tool axis


def check_clear_of_box(model: ArmModel, steps: list[Step], T_box: np.ndarray, part_len_m: float, label: str,
                       samples: int = 10, holding: bool = False) -> None:
    """Every pose of these steps keeps the jaws, the wrist and (while holding) the part's far end outside the box
    sitting at T_box (plus BOX_CLEAR_MM). Joint moves are checked along their interpolation."""
    from ..motion.program import CheckHolding, Grip, JointMove, LineMove

    R_inv, t = T_box[:3, :3].T, T_box[:3, 3]
    m = BOX_CLEAR_MM / 1000
    lo, hi = np.array([-BOX_W / 2 - m, -BOX_W / 2 - m, -1.0]), np.array([BOX_W / 2 + m, BOX_W / 2 + m, BOX_H + m])
    prev = None  # `holding`: already holding the part at the start

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
                    raise PathError(f"{label}: '{st.label}' would hit the box at the pack spot", pt)
        if isinstance(st, LineMove):
            prev = st.path[-1]
        elif isinstance(st, JointMove) or isinstance(st, _Start):
            prev = st.pose


def plan_pack_order(model: ArmModel, cfg, cal, order, box, current: dict[str, float],
                    obstacles: list | None = None) -> PackPlan:
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

    def failed(e: PathError, sub_steps=None) -> PathError:
        """The failure with the whole order's path up to it (sub-plan steps continue from the current end)."""
        return e.wrap(str(e), steps + list(sub_steps if sub_steps is not None else (e.steps or [])), current)

    T_box = box_pose(box, "pack", cfg, model.pan_xy)
    # between the steps (after the box move, after each part, at the end) the arm folds back to the rest pose
    # (`cfg.rest()`, taught `sleep`), checked clear of the boxes, the tray and the desk
    rest = cfg.rest() if hasattr(cfg, "rest") else None
    from .desk import check_desk_clear
    from .geometry import box_block

    desk = cfg.desk_plane() if hasattr(cfg, "desk_plane") else None
    around = [*(obstacles or []), box_block(T_box, f"box {box.id}")]

    def to_rest(label: str) -> None:
        nonlocal pose
        try:
            back = rest_move(model, pose, rest, around, desk, label)
        except PathError as e:
            raise failed(e, []) from e
        if back:
            steps.extend(back)
            pose = dict(back[-1].pose)

    try:
        if box.location != "pack":
            mv = plan_box_move(model, box, box.location, "pack", pose, cfg, obstacles)
            steps += mv.steps
            pose = last_pose(mv.steps)
        b0 = ProgramBuilder(model, pose)
        retreat_up(model, b0, T_box, "up and away from the box")
    except PathError as e:
        raise failed(e) from e
    steps += b0.steps
    pose = b0.pose
    to_rest(f"box {box.id} at the pack spot")
    lengths = {t.sku: t.length_mm for t in cfg.tray.types}
    types = {t.sku: t for t in cfg.tray.types}
    items, item_steps = [], []
    for i, slot in enumerate(order.slots):
        what = f"item {i + 1} ({slot})"
        try:
            pick = plan_pocket_pick(model, cfg.tray, cfg.gripper, cal, slot, pose)
        except PathError as e:
            raise failed(e.wrap(f"{what}: {e}")) from e
        try:
            check_clear_of_box(model, [_Start(pose), *pick.steps], T_box, lengths.get(pick.sku, 60.0) / 1000,
                               f"item {i + 1} ({pick.sku} from {slot})")
        except PathError as e:
            raise failed(e, pick.steps) from e
        b = ProgramBuilder(model, last_pose(pick.steps))
        try:
            drop = plan_drop(model, b, T_box, lengths.get(pick.sku, 60.0), pick.open_value,
                             f"item {i + 1} ({pick.sku} from {slot})",
                             part_height_mm=part_height_mm(types.get(pick.sku), pick.grip_mm),
                             jaw_offset_mm=pick.side_offset_mm)
        except PathError as e:
            raise failed(e, list(pick.steps) + list(e.steps or [])) from e
        base = len(steps)
        steps += pick.steps + b.steps
        item_steps.append({"start": base, "taken": base + len(pick.steps), "end": len(steps)})
        pose = b.pose
        to_rest(what)
        items.append({"pocket": slot, "sku": pick.sku, "pitch_deg": pick.pitch_deg,
                      "drop_tilt_deg": round(drop.pitch_deg, 1), "drop_roll": round(drop.roll, 1),
                      "drop_roll_change": round(drop.roll_change, 1), "drop_fits_opening": drop.fits_opening})
    try:
        out = plan_box_move(model, box, "pack", "output", pose, cfg, obstacles, rest=rest)
    except PathError as e:
        raise failed(e) from e
    steps += out.steps
    check_desk_clear(model, current, steps, desk, f"order {order.id}")  # picks + drops too, not just box moves
    if not any(isinstance(s, OpenGripper) for s in steps):
        raise AssertionError("no release step")  # can't happen: every drop and box move opens
    return PackPlan(order.id, box.id, steps, items, item_steps=item_steps)
