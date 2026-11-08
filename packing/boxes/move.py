"""Plan moving a box by its tab between two spots, as a motion program (packing/motion/program.py).

The grip is horizontal (user, 2026-11-07): gripper level, pointing away from the arm along the tab, jaws closing
up/down on the 2 mm plate.

  open → free move to `approach_mm` back from the grasp (toward the arm, level) → in LEVEL along the tab → grip
  → straight UP (`carry_mm`, or higher — CARRY_HEIGHTS_MM — until the carry clears everything: uses the vertical
  space over the tray / other boxes) → joint-space carry to above the destination (checked in 3D: never dips, box
  tilt ≤ MAX_CARRY_TILT_DEG, the box block ≥ CLEARANCE from the other boxes and the tray + its stand)
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

from ..motion.kinematics import ArmModel, PathError
from ..motion.program import ProgramBuilder, Step, describe
from ..tray.pick import AxisIK, best_targets
from .geometry import TAB_T, Block, blocks_intersect, box_block, box_frame, box_tilt_deg, mirror
from .store import Box

MAX_CARRY_TILT_DEG = 30.0  # box tilt while carrying (user: tilting is fine; a packed box may hold parts)
CARRY_HEIGHTS_MM = (60.0, 100.0, 140.0, 180.0, 220.0)  # tried after `carry_mm` when a carry would hit something
CLEARANCE = 0.010  # carried box kept this far from other boxes / the tray (m)
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
    """4x4 box pose at a location: supply = calibrated, pack = `boxes.pack_spot_mm`, output = supply mirrored."""
    if location == "supply":
        return box.supply
    if location == "output":
        return mirror(box.supply, pan_xy)
    if location == "pack":
        x, y = cfg.boxes.pack_spot_mm
        return box_frame((x / 1000, y / 1000), float(box.supply[2, 3]), pan_xy)
    raise ValueError(location)


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


def check_carry(model: ArmModel, box: Box, a: dict, b: dict, z_grasp: float, lift_m: float,
                obstacles: list[Block], samples: int = 30) -> float:
    """The joint-space carry from pose a to b: the box stays up (tool never below half the lift above the grasp
    height), its tilt stays ≤ MAX_CARRY_TILT_DEG, and the box (3D block) never comes within CLEARANCE of an
    obstacle (other boxes, the tray and its stand). Returns the largest box tilt. Raises PathError."""
    worst = 0.0
    tool_in_box_inv = np.linalg.inv(box.tool_in_box)
    for i in range(samples + 1):
        s = i / samples
        T = model.fk({j: a[j] + (b[j] - a[j]) * s for j in a})
        if T[2, 3] < z_grasp + lift_m / 2:
            raise PathError(f"carrying box {box.id} would dip to {(T[2, 3] - z_grasp) * 1000:.0f} mm above its "
                            f"grasp height", T[:3, 3])
        tilt = box_tilt_deg(T, box.tool_in_box)
        worst = max(worst, tilt)
        if tilt > MAX_CARRY_TILT_DEG:
            raise PathError(f"carrying box {box.id} would tilt it {tilt:.0f}° (max {MAX_CARRY_TILT_DEG:.0f}°)", T[:3, 3])
        carried = box_block(T @ tool_in_box_inv)
        for ob in obstacles:
            if blocks_intersect(carried, ob, CLEARANCE / 2):
                raise PathError(f"carrying box {box.id} would hit {ob.name} ({s:.0%} of the way)", T[:3, 3])
    return worst


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
    low as possible: `carry_mm`, then higher (CARRY_HEIGHTS_MM) until it clears everything.
    Raises PathError if any part is unreachable, over a joint limit, too tilted or would collide."""
    pan_xy = model.pan_xy
    obstacles = obstacles or []
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
    open_value = round(g.open_for(TAB_T * 1000), 1)
    # no "missed it" width check for boxes (user): the tab plate is 2 mm and a box wall ~0.5 mm — the jaws close
    # almost fully either way, so a width threshold can't tell a grip from a miss
    min_width = 0.0
    ik = AxisIK(model)

    b = ProgramBuilder(model, current)
    b.open_gripper(f"open for the tab ({open_value})", open_value)
    back = T_src[:3, 3] - _level(T_src[:3, 2]) * bc.approach_mm / 1000
    seeds = ([box.joints] if box.joints else []) + reach_seeds(model, back)
    targets, poses = best_targets(model, ik, back, T_src[:3, 2], T_src[:3, 0], seeds)
    move = b.joint_to(targets[0], f"in front of box {box.id}'s tab", seeds=poses, targets=targets[:1])
    travel = max(abs(move.pose[j] - b.start[j]) for j in move.pose)
    b.line_to(T_src[:3, 3], bc.down_speed_mm_s, "in level along the tab", orientation_weight=OW)
    b.grip(min_width, f"grip box {box.id}")
    gripped = b.mark()
    heights = [bc.carry_mm] + [h for h in CARRY_HEIGHTS_MM if h > bc.carry_mm]
    errors: list[PathError] = []
    for lift_mm in heights:  # use vertical space: lowest carry that clears everything
        b.reset(gripped)
        try:
            b.line_by(up * lift_mm / 1000, bc.line_speed_mm_s, f"lift straight up {lift_mm:.0f} mm",
                      orientation_weight=OW)
            b.check_holding(min_width, "check holding the box")
            lifted = b.pose
            dst_up = T_dst[:3, 3] + up * lift_mm / 1000
            targets, poses = best_targets(model, ik, dst_up, T_dst[:3, 2], T_dst[:3, 0],
                                          [lifted, *reach_seeds(model, dst_up)])
            carry = b.joint_to(targets[0], f"carry to above {LOCATION_NAMES[dst]}", seeds=poses, targets=targets[:1],
                               speed=cfg.motion.speed / 2)
            worst = check_carry(model, box, lifted, carry.pose, float(T_src[2, 3]), lift_mm / 1000, obstacles)
            break
        except PathError as e:
            errors.append(e.wrap(f"{lift_mm:.0f} mm: {e}", e.steps or list(b.steps), b.start))
    else:  # every height's reason; the path/point shown are the lowest attempt's (usually the real cause)
        raise errors[0].wrap("no carry height works — " + "; ".join(str(e) for e in errors))
    land = b.line_to(T_dst[:3, 3], bc.down_speed_mm_s, f"straight down {lift_mm:.0f} mm to the table (same height as picked)",
                     orientation_weight=OW)
    land_tilt = box_tilt_deg(model.fk(land.path[-1]), box.tool_in_box)
    if land_tilt > MAX_LAND_TILT_DEG:
        raise PathError(f"setting box {box.id} down at {LOCATION_NAMES.get(dst, dst)} would tilt it {land_tilt:.0f}° "
                        f"(max {MAX_LAND_TILT_DEG:.0f}°)", T_dst[:3, 3], list(b.steps), b.start)
    b.open_gripper("release the box", open_value)
    b.line_by(-_level(T_dst[:3, 2]) * bc.approach_mm / 1000, bc.line_speed_mm_s, "back out level along the tab",
              orientation_weight=OW)
    return BoxMovePlan(box.id, src, dst, b.steps, travel, lift_mm, tilt_deg=land_tilt, max_carry_tilt_deg=worst)
