"""Plan moving a box by its tab between two spots, as a motion program (packing/motion/program.py).

  open (tab width + margin) → free move to `approach_mm` above the grasp → straight DOWN onto the tab → grip
  → straight UP `carry_mm` (clears the tray and neighbouring boxes before any sideways motion) → straight line
  to above the destination → straight DOWN to exactly the grasp height (the box lands on the table where it
  will be picked up again) → open → straight UP clear of the tab.

The grasp is the calibration grasp (`Box.tool_in_box`) moved with the box: destination tool pose =
destination box pose · tool_in_box, at the same height. Box poses keep their tab facing the swivel, and the
straight carry line holds the tool orientation relative to the arm's plane, so the 5-DOF arm can follow it.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable

import numpy as np

from ..motion.kinematics import ArmModel
from ..motion.program import ProgramBuilder, Step, describe
from ..tray.pick import AxisIK, best_targets
from ..motion.kinematics import PathError
from .geometry import TAB_W, box_frame, footprint, mirror, overlaps
from .store import Box

MAX_CARRY_TILT_DEG = 12.0
MAX_LAND_TILT_DEG = 5.0  # gripper tilt when setting a box down (more = it lands on one edge)
OW = 0.01  # box lines: position (straight up/down, exact landing) beats holding the exact tool yaw
LOCATION_NAMES = {"supply": "its supply spot (left)", "pack": "the pack spot", "output": "its output spot (right)"}


def down_seeds(model: ArmModel, p: np.ndarray) -> list[dict[str, float]]:
    """IK seeds for reaching down at point p: pan toward p, elbow bent, wrist pointing down, both rolls."""
    from math import degrees

    az = degrees(model.azimuth(p))
    # degrees -> calibrated pan units; the sign comes from the model (left = negative pan here)
    zero = dict.fromkeys(["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"], 0.0)
    a0 = model.azimuth(model.tip(zero))
    a1 = model.azimuth(model.tip({**zero, "shoulder_pan": 10.0}))
    per_unit = degrees(a1 - a0) / 10.0
    pan = (az - degrees(a0)) / per_unit
    return [{"shoulder_pan": pan, "shoulder_lift": lift, "elbow_flex": elbow, "wrist_flex": wrist, "wrist_roll": roll}
            for lift, elbow, wrist in ((-20, 30, 70), (0, 50, 60), (-40, 10, 80), (10, 70, 40))
            for roll in (0.0, 50.0, -50.0)]


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
    tilt_deg: float = 0.0  # GotoRunner reports it

    @property
    def label(self) -> str:
        return f"move box {self.box_id} to {LOCATION_NAMES.get(self.dst, self.dst)}"

    def summary(self) -> dict:
        return {"box_id": self.box_id, "from": self.src, "to": self.dst, "travel_units": round(self.travel_units, 1),
                "carry_mm": self.carry_mm, "land_tilt_deg": round(self.tilt_deg, 1), "steps": describe(self.steps)}


def check_carry(model: ArmModel, box: Box, a: dict, b: dict, T_src: np.ndarray, T_dst: np.ndarray,
                obstacles: list[tuple[str, np.ndarray]], bc, samples: int = 24) -> None:
    """The joint-space carry from pose a to b: the box stays up (tool never below half the lift above the
    grasp height), level (tool within MAX_CARRY_TILT_DEG of the grasp's), and its footprint never overlaps an
    obstacle (other boxes, the tray). Raises PathError."""
    z0 = float(T_src[2, 3])
    tool_in_box_inv = np.linalg.inv(box.tool_in_box)
    for i in range(samples + 1):
        s = i / samples
        q = {j: a[j] + (b[j] - a[j]) * s for j in a}
        T = model.fk(q)
        if T[2, 3] < z0 + bc.carry_mm / 2000:
            raise PathError(f"carrying box {box.id} would dip to {(T[2, 3] - z0) * 1000:.0f} mm above its grasp height")
        tilt = np.degrees(np.arccos(np.clip(T[:3, 2] @ T_src[:3, 2], -1, 1)))
        if tilt > MAX_CARRY_TILT_DEG:
            raise PathError(f"carrying box {box.id} would tilt it {tilt:.0f}°")
        fp = footprint(T @ tool_in_box_inv, margin=0.005)
        for name, other in obstacles:
            if overlaps(fp, other):
                raise PathError(f"carrying box {box.id} would hit {name} ({s:.0%} of the way)")


def plan_box_move(model: ArmModel, box: Box, src: str, dst: str, current: dict[str, float], cfg,
                  obstacles: list[tuple[str, np.ndarray]] | None = None) -> BoxMovePlan:
    """`obstacles`: (name, footprint Nx2) of other boxes / the tray, for the carry check.
    Raises PathError if any part is unreachable, over a joint limit or would collide."""
    T_land = box_pose(box, dst, cfg, model.pan_xy)
    for name, other in obstacles or []:
        if overlaps(footprint(T_land, margin=0.005), other):
            raise PathError(f"box {box.id} at {LOCATION_NAMES.get(dst, dst)} would overlap {name}")
    pan_xy = model.pan_xy
    bc, g = cfg.boxes, cfg.gripper
    T_src = box_pose(box, src, cfg, pan_xy) @ box.tool_in_box
    T_dst = box_pose(box, dst, cfg, pan_xy) @ box.tool_in_box
    up = np.array([0.0, 0.0, 1.0])
    open_value = round(g.open_for(TAB_W * 1000), 1)
    min_width = round(g.units_for(TAB_W * 1000 * 0.6), 1)  # closed much tighter than the tab = missed it

    b = ProgramBuilder(model, current)
    b.open_gripper(f"open for the tab ({open_value})", open_value)
    above = T_src[:3, 3] + up * bc.approach_mm / 1000
    seeds = ([box.joints] if box.joints else []) + down_seeds(model, above)
    # best reachable orientation at that point (exact position): approach axis + jaw direction of the grasp
    ik = AxisIK(model)
    targets, poses = best_targets(model, ik, above, T_src[:3, 2], T_src[:3, 0], seeds)
    move = b.joint_to(targets[0], f"above box {box.id}'s tab", seeds=poses, targets=targets[:1])
    travel = max(abs(move.pose[j] - b.start[j]) for j in move.pose)
    b.line_to(T_src[:3, 3], bc.down_speed_mm_s, "straight down onto the tab", orientation_weight=OW)
    b.grip(min_width, f"grip box {box.id}")
    b.line_by(up * bc.carry_mm / 1000, bc.line_speed_mm_s, f"lift straight up {bc.carry_mm:.0f} mm", orientation_weight=OW)
    b.check_holding(min_width, "check holding the box")
    # carry: a joint move between the two lifted poses (mostly the base swivelling), checked sample by sample
    lifted = b.pose
    dst_up = T_dst[:3, 3] + up * bc.carry_mm / 1000
    targets, poses = best_targets(model, ik, dst_up, T_dst[:3, 2], T_dst[:3, 0], [lifted, *down_seeds(model, dst_up)])
    carry = b.joint_to(targets[0], f"carry to above {LOCATION_NAMES[dst]}", seeds=poses, targets=targets[:1],
                       speed=cfg.motion.speed / 2)
    check_carry(model, box, lifted, carry.pose, T_src, T_dst, obstacles or [], bc)
    land = b.line_to(T_dst[:3, 3], bc.down_speed_mm_s, "straight down to the table (same height as picked)",
                     orientation_weight=OW)
    land_tilt = float(np.degrees(np.arccos(np.clip(-model.fk(land.path[-1])[2, 2], -1, 1))))
    if land_tilt > MAX_LAND_TILT_DEG:
        raise PathError(f"setting box {box.id} down at {LOCATION_NAMES.get(dst, dst)} would tilt it {land_tilt:.0f}° "
                        f"(max {MAX_LAND_TILT_DEG:.0f}°) — too close to the base?")
    b.open_gripper("release the box", open_value)
    b.line_by(up * bc.approach_mm / 1000, bc.line_speed_mm_s, "jaws straight up off the tab", orientation_weight=OW)
    return BoxMovePlan(box.id, src, dst, b.steps, travel, bc.carry_mm, tilt_deg=land_tilt)
