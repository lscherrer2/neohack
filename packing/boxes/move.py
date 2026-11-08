"""Plan moving a box by its tab between two spots, as a motion program (packing/motion/program.py).

The grip is horizontal (user, 2026-11-07): gripper level, pointing away from the arm along the tab, jaws closing
up/down on the 2 mm plate.

  open → free move to APPROACH_UP_M above the spot `approach_mm` back from the grasp (toward the arm) → straight
  DOWN to tab height → in LEVEL along the tab → grip
  → straight UP `carry_mm` → carry, checked in 3D (box block ≥ CLEARANCE from the other boxes and the tray + its
  stand, the arm ≥ ARM_CLEARANCE, never dips, box tilt ≤ MAX_CARRY_TILT_DEG):
      direct — swing to above the destination (lifted higher first if the swing passes something low), or
      arc OVER — arc up and IN toward the swivel (the arm reaches far higher close in), swing around high,
      arc down and out to above the destination
  → straight DOWN to exactly the grasp height (the box lands where it will be picked up again; box tilt ≤
  MAX_LAND_TILT_DEG) → open → back out LEVEL along the tab.

The grasp is the calibration grasp (`Box.tool_in_box`) moved with the box: destination tool pose = destination box
pose · tool_in_box, at the same height. Box poses keep their tab facing the swivel; with 5 DOF the tool yaw
follows the pan, so the arm can reach that grasp anywhere around it. Tilts are measured on the BOX (tool pose ·
inverse grasp), so the checks hold for any grip.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from math import degrees
from typing import Callable

import numpy as np

from ..motion.kinematics import POSITION_TOL_M, ArmModel, PathError
from ..motion.program import ProgramBuilder, Step, describe
from ..tray.pick import AxisIK, best_targets
from .geometry import (TAB_T, Block, block_footprint, block_top, blocks_intersect, box_block, box_frame, box_tilt_deg,
                       footprint, mirror, overlaps, point_in_block, with_fixed_jaw)
from .store import Box

MAX_CARRY_TILT_DEG = 30.0  # box tilt while carrying (user: tilting is fine; a packed box may hold parts)
APPROACH_UP_M = 0.05  # the free move goes to this far above the spot in front of the tab, then straight down
DIRECT_MAX_MM = 80.0  # straight lifts above the box spot beyond this: use the arc instead (reach is low out there)
ARC_TOP_M = 0.28  # highest box-bottom height tried for the arc (level grip reaches tool ~35 cm up at 12–27 cm out)
ARC_RADII_M = (0.22, 0.19, 0.25, 0.16)  # tool point distance from the swivel while swinging high, tried in order
CLEARANCE = 0.010  # carried box kept this far from other boxes / the tray (m)
ARM_CLEARANCE = 0.015  # arm links (sampled along their center lines) kept this far from obstacles (m)
OVER_MARGIN = 0.04  # a carry whose box passes within this of an obstacle's outline on the desk goes OVER it
OVER_CLEAR = 0.03  # ...with the box bottom this far above the obstacle's top
ARM_CHAIN = ["upper_arm_link", "lower_arm_link", "wrist_link", "gripper_link", "gripper_frame_link",
             "moving_jaw_so101_v1_link"]
MAX_LAND_TILT_DEG = 5.0  # box tilt when setting it down (more = it lands on one edge, off its spot)
OW = 0.05  # box lines: position (level in/out, straight up/down, exact landing) beats holding the exact tool yaw
LOCATION_NAMES = {"supply": "its supply spot (left)", "pack": "the pack spot", "output": "its output spot (right)"}
BODY = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]


def _pan_units(model: ArmModel, p: np.ndarray) -> float:
    """shoulder_pan (calibrated units) that points the arm at p; the sign comes from the model (left = negative)."""
    zero = dict.fromkeys(BODY, 0.0)
    a0 = model.azimuth(model.tip(zero))
    per_unit = degrees(model.azimuth(model.tip({**zero, "shoulder_pan": 10.0})) - a0) / 10.0
    return (degrees(model.azimuth(p)) - degrees(a0)) / per_unit


def reach_seeds(model: ArmModel, p: np.ndarray) -> list[dict[str, float]]:
    """IK seeds for reaching point p: pan toward it; level-gripper and pointing-down arm shapes; several rolls."""
    pan = _pan_units(model, p)
    shapes = ((10, 30, -30), (20, 50, -60), (0, 10, 0), (30, 70, -90), (21, 80, -77),  # gripper level, low
              (-20, 30, 70), (0, 50, 60), (-40, 10, 80),  # reaching down, near
              (-40, 40, 0), (-40, 0, 45), (-40, 80, 45), (-40, -40, 90), (-40, 40, 90))  # pointing down, far out
    return [{"shoulder_pan": pan, "shoulder_lift": lift, "elbow_flex": elbow, "wrist_flex": wrist, "wrist_roll": roll}
            for lift, elbow, wrist in shapes for roll in (0.0, 60.0, -60.0)]


down_seeds = reach_seeds  # used by the drop / retreat planning (pack.py)


def box_pose(box: Box, location: str, cfg, pan_xy: np.ndarray) -> np.ndarray:
    """4x4 box pose at a location: supply = calibrated, pack = `boxes.pack_spot_mm`, output = supply mirrored.
    Heights away from the calibrated spot follow the desk plane if one is calibrated (base / desk tilt)."""
    from .desk import height_at

    if location == "supply":
        return box.supply
    if location == "output":
        xy = mirror(box.supply, pan_xy)[:2, 3]
    elif location == "pack":
        xy = np.array(cfg.boxes.pack_spot_mm, float) / 1000
    else:
        raise ValueError(location)
    plane = cfg.desk_plane() if hasattr(cfg, "desk_plane") else None
    z = height_at(plane, box.supply[:2, 3], xy, float(box.supply[2, 3]))
    return box_frame((float(xy[0]), float(xy[1])), z, pan_xy)


@dataclass
class BoxMovePlan:
    box_id: int
    src: str
    dst: str
    steps: list[Step]
    travel_units: float
    carry_mm: float
    on_done: Callable[[], None] | None = field(default=None, repr=False)  # e.g. record the new location
    tilt_deg: float = 0.0  # box tilt at the set-down (GotoRunner reports it)
    max_carry_tilt_deg: float = 0.0

    @property
    def label(self) -> str:
        return f"move box {self.box_id} to {LOCATION_NAMES.get(self.dst, self.dst)}"

    def summary(self) -> dict:
        return {"box_id": self.box_id, "from": self.src, "to": self.dst, "travel_units": round(self.travel_units, 1),
                "carry_mm": self.carry_mm, "land_tilt_deg": round(self.tilt_deg, 1),
                "max_carry_tilt_deg": round(self.max_carry_tilt_deg, 1), "steps": describe(self.steps)}


DIP_TOL = 0.005  # a guided carry may sag this much below its lower end (m)


def check_path(model: ArmModel, box: Box, path: list[dict], obstacles: list[Block], label: str, desk=None) -> float:
    """A guided carry (dense joint path): the box (3D block) ≥ CLEARANCE from obstacles, the arm ≥ ARM_CLEARANCE,
    box tilt ≤ MAX_CARRY_TILT_DEG, the tool never below the lower end of the path − DIP_TOL, and (with a desk
    plane) no box corner below the desk + BOX_CLEAR_M. Returns the largest box tilt. Raises PathError."""
    from .desk import box_below_desk

    tool_in_box_inv = np.linalg.inv(box.tool_in_box)
    z_ends = [float(model.fk(path[0])[2, 3]), float(model.fk(path[-1])[2, 3])]
    floor = min(z_ends) - DIP_TOL
    worst = 0.0
    for k, q in enumerate(path):
        T = model.fk(q)
        s = k / max(1, len(path) - 1)
        if T[2, 3] < floor:
            raise PathError(f"{label}: box {box.id} would sag {(min(z_ends) - T[2, 3]) * 1000:.0f} mm "
                            f"({s:.0%} of the way)", T[:3, 3])
        tilt = box_tilt_deg(T, box.tool_in_box)
        worst = max(worst, tilt)
        if tilt > MAX_CARRY_TILT_DEG:
            raise PathError(f"{label}: box {box.id} would tilt {tilt:.0f}° (max {MAX_CARRY_TILT_DEG:.0f}°)", T[:3, 3])
        carried = box_block(T @ tool_in_box_inv)
        if (below := box_below_desk(T @ tool_in_box_inv, desk)) > 0:
            raise PathError(f"{label}: a corner of box {box.id} would come within {below * 1000:.0f} mm of touching "
                            f"the desk ({s:.0%} of the way)", T[:3, 3])
        for ob in obstacles:
            if blocks_intersect(carried, ob, CLEARANCE / 2):
                raise PathError(f"{label}: box {box.id} would hit {ob.name} ({s:.0%} of the way)", T[:3, 3])
        check_arm_clear(model, q, obstacles, f"{label}", s)
    return worst


def arm_points(model: ArmModel, q: dict) -> list[np.ndarray]:
    """Points along the arm's center line (link origins + midpoints), shoulder to jaw tip."""
    frames = model.link_frames(q)
    pts = [frames[n][:3, 3] for n in ARM_CHAIN]
    return pts + [(pts[i] + pts[i + 1]) / 2 for i in range(len(pts) - 1)]


def check_arm_clear(model: ArmModel, q: dict, obstacles: list[Block], what: str, s: float | None = None) -> None:
    """The arm itself (not just the box) stays ARM_CLEARANCE away from every obstacle at pose q."""
    for pt in arm_points(model, q):
        for ob in obstacles:
            if point_in_block(pt, ob, ARM_CLEARANCE):
                where = f" ({s:.0%} of the way)" if s is not None else ""
                raise PathError(f"{what}: the arm would hit {ob.name}{where}", pt)


def over_height_mm(model: ArmModel, box: Box, a: dict, b: dict, z_box_bottom: float, obstacles: list[Block],
                   samples: int = 30) -> tuple[float, str | None, float]:
    """Lift (mm above the grasp) needed so the box goes OVER every obstacle whose desk outline the carry from a to
    b passes within OVER_MARGIN of (box bottom ≥ its top + OVER_CLEAR). 0 if it passes nothing."""
    tool_in_box_inv = np.linalg.inv(box.tool_in_box)
    need, what, what_top = 0.0, None, 0.0
    outlines = [(ob, block_footprint(ob), block_top(ob)) for ob in obstacles]
    for i in range(samples + 1):
        s = i / samples
        T_box = model.fk({j: a[j] + (b[j] - a[j]) * s for j in a}) @ tool_in_box_inv
        fp = footprint(T_box, margin=OVER_MARGIN)
        for ob, outline, top in outlines:
            if overlaps(fp, outline) and top + OVER_CLEAR - z_box_bottom > need:
                need, what, what_top = top + OVER_CLEAR - z_box_bottom, ob.name, top
    return need * 1000, what, what_top * 1000


def too_close(T_box: np.ndarray, pan_xy: np.ndarray, cfg) -> str | None:
    """Plain-language reason if a box at T_box is too close to the swivel for the level tab grip, else None."""
    r = float(np.linalg.norm(T_box[:2, 3] - pan_xy))
    need = cfg.boxes.min_center_mm / 1000
    if r >= need:
        return None
    return (f"is {r * 100:.1f} cm from the arm's swivel; it needs ≥ {need * 100:.1f} cm (closer, the arm can't come in "
            f"/ back out level along the tab) — slide it {(need - r) * 100 + 0.5:.0f} cm further out")


def _level(v: np.ndarray) -> np.ndarray:
    """Horizontal part of a direction, normalized (come in / back out level along the tab)."""
    h = np.array([v[0], v[1], 0.0])
    return h / np.linalg.norm(h)


def plan_box_move(model: ArmModel, box: Box, src: str, dst: str, current: dict[str, float], cfg,
                  obstacles: list[Block] | None = None) -> BoxMovePlan:
    """`obstacles`: 3D blocks of the other boxes / the tray, for the landing and carry checks. The carry goes as
    low as possible (`carry_mm`); if the swing passes near an obstacle the box goes OVER it — by a higher straight
    lift if that's low, else by arcing up and in toward the swivel, swinging high and arcing back down.
    Raises PathError if any part is unreachable, over a joint limit, too tilted or would collide."""
    import dataclasses

    from .desk import check_desk_clear

    pan_xy = model.pan_xy
    obstacles = obstacles or []
    desk = cfg.desk_plane() if hasattr(cfg, "desk_plane") else None
    # grip with the fixed jaw where configured (default under the plate: wrist roll ~0, far from its limit)
    grip = with_fixed_jaw(box.tool_in_box, cfg.boxes.fixed_jaw)
    flipped = grip is not box.tool_in_box
    box = dataclasses.replace(box, tool_in_box=grip, joints={} if flipped else box.joints)  # (capture joints: old grip)
    T_land_box = box_pose(box, dst, cfg, pan_xy)
    for where in (src, dst):
        if why := too_close(box_pose(box, where, cfg, pan_xy), pan_xy, cfg):
            raise PathError(f"box {box.id} at {LOCATION_NAMES.get(where, where)} {why}, then remove and re-add it",
                            box_pose(box, where, cfg, pan_xy)[:3, 3])
    for ob in obstacles:
        if blocks_intersect(box_block(T_land_box), ob, 0.0025):
            raise PathError(f"box {box.id} at {LOCATION_NAMES.get(dst, dst)} would overlap {ob.name}", T_land_box[:3, 3])
    bc, g = cfg.boxes, cfg.gripper
    T_src = box_pose(box, src, cfg, pan_xy) @ box.tool_in_box
    T_dst = T_land_box @ box.tool_in_box
    up = np.array([0.0, 0.0, 1.0])
    # jaws open to `jaw_gap_mm` at the fingertips for the approach and the release / back-out (user: 2x the plate +
    # margin — slack for imprecise reaching far out)
    open_value = round(g.open_for(max(bc.jaw_gap_mm - g.open_margin_mm, TAB_T * 1000)), 1)
    # no "missed it" width check for boxes (user): the tab plate is 2 mm and a box wall ~0.5 mm — the jaws close
    # almost fully either way, so a width threshold can't tell a grip from a miss
    min_width = 0.0
    ik = AxisIK(model)

    b = ProgramBuilder(model, current)
    b.open_gripper(f"open for the tab ({open_value})", open_value)
    # approach FROM ABOVE: free move to above the spot in front of the tab (checked: the arm clears this box, the
    # tray and the other boxes), straight down to tab height, then in level along the tab — the jaws never swoop
    # in from below
    back = T_src[:3, 3] - _level(T_src[:3, 2]) * bc.approach_mm / 1000
    above_back = back + up * APPROACH_UP_M
    seeds = ([box.joints] if box.joints else []) + reach_seeds(model, above_back)
    targets, poses = best_targets(model, ik, above_back, T_src[:3, 2], T_src[:3, 0], seeds)
    start_pose = dict(b.pose)
    move = b.joint_to(targets[0], f"above the spot in front of box {box.id}'s tab", seeds=poses, targets=targets[:1],
                      speed=bc.free_speed)
    travel = max(abs(move.pose[j] - b.start[j]) for j in move.pose)
    this_box = box_block(box_pose(box, src, cfg, pan_xy), f"box {box.id} itself")
    def near_this_box(q: dict) -> bool:
        return any(point_in_block(pt, this_box, ARM_CLEARANCE) for pt in arm_points(model, q))

    clear_of_it = not near_this_box(start_pose)  # starting right next to it (just let go of it): ignore it until clear
    for k in range(1, 21):  # the free move: the arm stays clear of everything, including the box it goes for
        q = {j: start_pose[j] + (move.pose[j] - start_pose[j]) * k / 20 for j in start_pose}
        clear_of_it = clear_of_it or not near_this_box(q)
        check_arm_clear(model, q, [*obstacles, this_box] if clear_of_it else obstacles, f"approaching box {box.id}",
                        k / 20)
    b.line_to(back, bc.down_speed_mm_s, f"straight down {APPROACH_UP_M * 1000:.0f} mm in front of the tab",
              orientation_weight=OW)
    b.line_to(T_src[:3, 3], bc.down_speed_mm_s, "in level along the tab", orientation_weight=OW)
    b.grip(min_width, f"grip box {box.id}")
    gripped = b.mark()
    z_bottom = float(box_pose(box, src, cfg, pan_xy)[2, 3])  # box bottom at the grasp (on the desk)
    z_off = float(T_src[2, 3]) - z_bottom  # tool point height above the box bottom
    # what does the swing pass near? (estimated at the lowest lift) -> how high the box bottom must go to pass OVER
    over, over_what, over_top = 0.0, None, 0.0
    try:
        probe = b.line_by(up * bc.carry_mm / 1000, bc.line_speed_mm_s, "probe", orientation_weight=OW)
        dst_low = T_dst[:3, 3] + up * bc.carry_mm / 1000
        _, poses = best_targets(model, ik, dst_low, T_dst[:3, 2], T_dst[:3, 0], [b.pose, *reach_seeds(model, dst_low)])
        over, over_what, over_top = over_height_mm(model, box, probe.path[-1], poses[0], z_bottom, obstacles)
    except PathError:
        pass
    errors: list[PathError] = []
    lift_mm = bc.carry_mm
    worst = 0.0

    def lift_up(mm: float) -> dict:
        b.line_by(up * mm / 1000, bc.line_speed_mm_s, f"lift straight up {mm:.0f} mm", orientation_weight=OW)
        b.check_holding(min_width, "check holding the box")
        return b.pose

    def solve(prev: dict, pt: np.ndarray, z_axis: np.ndarray, x_axis: np.ndarray) -> dict:
        targets, poses = best_targets(model, ik, pt, z_axis, x_axis, [prev, *reach_seeds(model, pt)])
        q = poses[0]
        if np.linalg.norm(model.tip(q) - pt) > POSITION_TOL_M:
            raise PathError(f"can't reach {np.round(pt * 1000, 1)} mm holding the box", pt)
        return q

    def guided(prev: dict, waypoints: list[np.ndarray], final_axes: np.ndarray | None, label: str) -> dict:
        """A carry through tool-point waypoints (box held level, tab facing the swivel), solved one after the other
        and played as ONE smooth dense path — so the box follows the waypoints (no sagging between far-apart
        poses). Checked: box + arm vs obstacles, box tilt, and the height never dips below the lower end."""
        sols, q = [], prev
        for k, pt in enumerate(waypoints):
            last = k == len(waypoints) - 1 and final_axes is not None
            u = np.array([*(pt[:2] - pan_xy), 0.0])
            u /= np.linalg.norm(u)
            q = solve(q, pt, final_axes[:3, 2] if last else u, final_axes[:3, 0] if last else T_src[:3, 0])
            sols.append(q)
        dense, a = [], prev
        for c in sols:  # fine joint interpolation between consecutive (close) waypoints
            dense += [{j: a[j] + (c[j] - a[j]) * t / 6 for j in a} for t in range(1, 7)]
            a = c
        nonlocal worst
        worst = max(worst, check_path(model, box, [prev, *dense], obstacles, label, desk))
        b.path(dense, bc.carry_speed_mm_s, label, max_joint_speed=bc.carry_joint_speed)
        return b.pose

    def ease_out(t: float) -> float:  # fast first: rise early / move out early
        return 1 - (1 - t) ** 2

    def ease_in(t: float) -> float:  # slow first
        return t * t

    az_src, az_dst = model.azimuth(T_src[:3, 3]), model.azimuth(T_dst[:3, 3])

    def polar(az: float, r: float, z: float) -> np.ndarray:
        return np.array([pan_xy[0] + r * np.cos(az), pan_xy[1] + r * np.sin(az), z])

    def r_of(p: np.ndarray) -> float:
        return float(np.linalg.norm(p[:2] - pan_xy))

    def swing(az0: float, az1: float, r0: float, r1: float, z: float, n_min: int = 3) -> list[np.ndarray]:
        n = max(n_min, int(np.ceil(abs(np.degrees(az1 - az0)) / 12)))
        return [polar(az0 + (az1 - az0) * k / n, r0 + (r1 - r0) * k / n, z) for k in range(1, n + 1)]

    # 1) direct: short straight lift (higher if the swing passes something low), swing at constant height around
    #    the swivel to above the destination. If the swing passes near something: only at the height that goes
    #    over it — never squeezing past it low.
    done = False
    for mm in [max(bc.carry_mm, over)]:
        if mm > DIRECT_MAX_MM:
            continue
        b.reset(gripped)
        try:
            lifted = lift_up(mm)
            p0, p1 = b.tip, T_dst[:3, 3] + up * mm / 1000
            pts = swing(az_src, az_dst, r_of(p0), r_of(p1), p0[2])
            pts[-1] = p1
            guided(lifted, pts, T_dst, f"carry around to above {LOCATION_NAMES[dst]}")
            lift_mm, done = mm, True
            break
        except PathError as e:
            errors.append(e.wrap(f"direct, {mm:.0f} mm up: {e}", e.steps or list(b.steps), b.start))

    # 2) arc OVER: short straight lift; arc up and IN toward the swivel (rising first, so it never sags near the
    #    box spot); swing around at that height; arc down and OUT (moving out first, descending last) to just above
    #    the destination; straight down
    if not done:
        lowest = max(z_bottom + bc.carry_mm / 1000 + 0.04, (over_top / 1000 + OVER_CLEAR) if over_what else 0.0)
        for zb in np.arange(lowest, ARC_TOP_M + 1e-9, 0.02):  # box bottom height while swinging, lowest first
            for r in ARC_RADII_M:  # tool point distance from the swivel while swinging
                b.reset(gripped)
                try:
                    lifted = lift_up(bc.carry_mm)
                    p0 = b.tip
                    zt, r0 = zb + z_off, r_of(p0)
                    up_arc = [polar(az_src, r0 + (r - r0) * ease_in(k / 6), p0[2] + (zt - p0[2]) * ease_out(k / 6))
                              for k in range(1, 7)]
                    p1 = T_dst[:3, 3] + up * bc.carry_mm / 1000
                    r1 = r_of(p1)
                    down_arc = [polar(az_dst, r + (r1 - r) * ease_out(k / 6), zt + (p1[2] - zt) * ease_in(k / 6))
                                for k in range(1, 7)]
                    down_arc[-1] = p1
                    # ONE continuous move (no stop-and-settle at the top between the up-arc, swing and down-arc)
                    guided(lifted, up_arc + swing(az_src, az_dst, r, r, zt) + down_arc, T_dst,
                           f"arc up, swing around high and arc down to above {LOCATION_NAMES[dst]}")
                    lift_mm, done = (zb - z_bottom) * 1000, True
                    break
                except PathError as e:
                    errors.append(e.wrap(f"arc at {zb * 1000:.0f} mm / {r * 100:.0f} cm: {e}", e.steps or list(b.steps),
                                         b.start))
            if done:
                break
    if not done:  # the path/point shown are the first attempt's
        why = (f"to go over {over_what} (top {over_top:.0f} mm) box {box.id} has to get its bottom ≥ "
               f"{over_top + OVER_CLEAR * 1000:.0f} mm up, and no direct lift or arc up to {ARC_TOP_M * 1000:.0f} mm "
               f"works" if over_what else "no carry works")
        raise errors[0].wrap(f"{why} — " + "; ".join(str(e) for e in errors[:4]) + (" …" if len(errors) > 4 else ""))
    down_mm = bc.carry_mm if b.steps[-1].label.startswith("arc") else lift_mm
    land = b.line_to(T_dst[:3, 3], bc.down_speed_mm_s, f"straight down {down_mm:.0f} mm to the table (same height as picked)",
                     orientation_weight=OW)
    land_tilt = box_tilt_deg(model.fk(land.path[-1]), box.tool_in_box)
    if land_tilt > MAX_LAND_TILT_DEG:
        raise PathError(f"setting box {box.id} down at {LOCATION_NAMES.get(dst, dst)} would tilt it {land_tilt:.0f}° "
                        f"(max {MAX_LAND_TILT_DEG:.0f}°)", T_dst[:3, 3], list(b.steps), b.start)
    b.open_gripper("release the box", open_value)
    b.line_by(-_level(T_dst[:3, 2]) * bc.approach_mm / 1000, bc.line_speed_mm_s, "back out level along the tab",
              orientation_weight=OW)
    check_desk_clear(model, b.start, b.steps, desk, f"moving box {box.id}")  # jaw tips / arm above the desk
    return BoxMovePlan(box.id, src, dst, b.steps, travel, lift_mm, tilt_deg=land_tilt, max_carry_tilt_deg=worst)
