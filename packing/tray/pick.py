"""Plan picking one part out of a tray pocket, as a motion program (packing/motion/program.py).

Everything comes from the tray calibration: pocket position, tray normal (= pull-out direction) and the grip
direction (the type's `grip_axis` in tray x/y, turned by the pocket yaw, mapped into the robot frame).

Program (one procedure for every type; the per-type differences are data in `tray.types`):
  open gripper (part width + margin) -> free move to the approach point (`protrusion + approach_mm` out along the normal)
  -> straight line in to `grasp_standoff_mm` off the tray face -> grip (stall detect, type `min_grip`;
  on a miss: open + back out) -> straight line out `pull_mm` along the normal -> check still holding
  -> straight line up `lift_mm` -> hold (drop spot: TBD).

Tool orientation: the jaws close along gripper_frame x, so tool x = the grip direction; tool z (approach axis)
points into the tray. The tray faces the arm, so pulling straight out heads back toward the base and a level
tool runs out of elbow travel (~4-10 cm short of 12 cm). Pitching the tool down lets the wrist take up the fold:
the planner tries pitch 0°, step, 2·step, … max and uses the smallest pitch that fits `pull_mm`, falling back to
the part's minimum (what clears its socket: length − protrusion + 10 mm; see PickSettings.for_type). The orientation is held during the pull, so the part still comes out
straight along the normal. `tilt_deg` = achieved angle between the tool axis and the inward normal.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from scipy.spatial.transform import Rotation

from ..motion.kinematics import ArmModel, PathError
from ..motion.program import JOINT_LIMIT, MAX_IK_ERR_M, LineMove, OpenGripper, ProgramBuilder, Step, describe

# Jaw geometry, measured from the SO-101 meshes (TheRobotStudio/SO-ARM100 wrist_roll_follower / moving_jaw STL,
# 2026-11-07), in the tool frame (gripper_frame_link): the FIXED jaw's inner face lies at x ~ 0 (through the
# tool point) and its body extends to +x; the MOVING jaw is on -x and opens toward -x (touches at gripper 0).
# So the part must sit on the -x side: tool point = part center + (face + radius + clearance) * tool x.
# The jaw tips stick out ~5-6 mm past the tool point along z (calibration touches used the tips: mostly cancels).
FIXED_JAW_FACE_MM = 0.0
IN_SPEED_MM_S = 30.0  # final approach to the part (2x the first 15, user 2026-11-07)
PULL_SPEED_MM_S = 40.0  # pulling the part out (2x 20)
LIFT_SPEED_MM_S = 60.0  # (2x 30)


@dataclass
class PickSettings:
    """Per-pick numbers, from the tray config (`tray:` in cell.yaml) and the item type."""

    grip_axis: str  # x | y (tray frame, before pocket yaw)
    min_grip: float  # gripper units: closing tighter than this = missed / dropped
    grip_mm: float = 0.0  # part width between the jaws
    open_value: float | None = None  # gripper value to open to (part width + margin); None = fully open
    fixed_jaw_offset_mm: float = 0.0  # tool point -> part center, toward the moving jaw (see FIXED_JAW_FACE_MM)
    fixed_jaw_side: str | None = None  # "+" / "-": fixed jaw only on that side along the grip axis (None = either)
    protrusion_mm: float = 10.0
    approach_mm: float = 30.0  # approach point = protrusion + this, out from the face
    grasp_standoff_mm: float = 3.81  # tool tip off the tray face when it closes (0.15 in)
    pull_mm: float = 150.0  # preferred straight pull-out
    extract_mm: float = 120.0  # minimum straight pull-out (per part: clears its socket)
    lift_mm: float = 100.0  # then straight up this far (shortened if out of reach, min 1/3)
    max_pitch_deg: float = 40.0
    pitch_step_deg: float = 10.0

    @classmethod
    def for_type(cls, tray_cfg, sku: str, gripper_cfg) -> PickSettings:
        t = next((t for t in tray_cfg.types if t.sku == sku), None)
        if t is None:
            raise KeyError(f"no tray type {sku!r}")
        margin = gripper_cfg.open_margin_mm if t.open_margin_mm is None else t.open_margin_mm
        return cls(grip_axis=t.grip_axis,
                   min_grip=t.min_grip if t.min_grip is not None else gripper_cfg.min_part_width,
                   grip_mm=t.grip_width_mm, open_value=round(gripper_cfg.open_for(t.grip_width_mm, margin), 1),
                   fixed_jaw_offset_mm=FIXED_JAW_FACE_MM + t.grip_width_mm / 2 + margin / 2,
                   fixed_jaw_side=t.fixed_jaw_side,
                   protrusion_mm=tray_cfg.protrusion_mm, approach_mm=tray_cfg.approach_mm,
                   grasp_standoff_mm=tray_cfg.grasp_standoff_mm,
                   # pull-out per part: what it takes to clear its socket (length − what sticks out + margin), with
                   # up to PULL_EXTRA_MM more where it fits; never more than the configured maximum
                   pull_mm=min(tray_cfg.pull_mm, _needed_pull(t, tray_cfg) + PULL_EXTRA_MM),
                   extract_mm=min(tray_cfg.pull_mm, _needed_pull(t, tray_cfg)), lift_mm=tray_cfg.lift_mm,
                   max_pitch_deg=tray_cfg.max_pitch_deg, pitch_step_deg=tray_cfg.pitch_step_deg)


PULL_MARGIN_MM = 10.0  # pulled this much further than the part can be deep in its socket (glue: 110 + 10 = the old 120)
PULL_EXTRA_MM = 40.0  # preferred extra pull-out when reachable


def _needed_pull(t, tray_cfg) -> float:
    """Straight pull-out that clears this part from its socket: it sits at most (length − protrusion) deep."""
    return max(t.length_mm - tray_cfg.protrusion_mm, 0.0) + PULL_MARGIN_MM


@dataclass
class PickPlan:
    pocket: str
    sku: str
    point_robot_m: list[float]
    steps: list[Step]
    pitch_deg: float  # downward pitch used (smallest that fit)
    tilt_deg: float  # achieved angle tool axis vs inward normal (pitch + 5-DOF sideways tilt)
    pull_mm: float
    lift_mm: float
    grasp_standoff_mm: float
    travel_units: float  # largest joint change of the free move
    pull_twist_deg: float = 0.0  # tool rotation during the pull-out (5-DOF: yaw follows the pan)
    jaw_off_deg: float = 0.0  # jaw closing axis vs the wanted grip direction at the grasp
    grip_mm: float = 0.0
    open_value: float | None = None
    side_offset_mm: float = 0.0  # tool point aimed this far from the part center, toward the fixed jaw

    @property
    def label(self) -> str:
        return f"pick {self.pocket}"

    def summary(self) -> dict:
        return {"pocket": self.pocket, "sku": self.sku, "point_robot_m": self.point_robot_m,
                "pitch_deg": self.pitch_deg, "tilt_deg": round(self.tilt_deg, 1), "pull_mm": self.pull_mm,
                "lift_mm": round(self.lift_mm, 1), "grasp_standoff_mm": self.grasp_standoff_mm,
                "travel_units": round(self.travel_units, 1), "pull_twist_deg": round(self.pull_twist_deg, 1),
                "jaw_off_deg": round(self.jaw_off_deg, 1), "grip_mm": self.grip_mm, "open_value": self.open_value,
                "side_offset_mm": round(self.side_offset_mm, 2),
                "steps": describe(self.steps)}


class AxisIK:
    """placo IK for a 5-DOF arm: tip at p (strong), tool z as close as possible to `inward` and jaws (tool x)
    along `close_dir` (soft). The solver itself finds the best achievable approach axis — no need to project
    into "the arm's plane" (which is not exactly through the tip: the tool point is 7.9 mm off the roll axis)."""

    Z_W, X_W = 1.0, 0.3  # tip position is a hard constraint; approach axis beats jaw direction

    def __init__(self, model: ArmModel):
        import placo

        self.model, kin = model, model.kin
        self.solver = placo.KinematicsSolver(kin.robot)
        self.solver.mask_fbase(True)
        self.pos = self.solver.add_position_task(kin.tip_frame, np.zeros(3))
        self.pos.configure("pick_pos", "hard")
        self.z = self.solver.add_axisalign_task(kin.tip_frame, np.array([0.0, 0.0, 1.0]), np.array([0.0, 0.0, 1.0]))
        self.z.configure("pick_z", "soft", self.Z_W)
        self.x = self.solver.add_axisalign_task(kin.tip_frame, np.array([1.0, 0.0, 0.0]), np.array([1.0, 0.0, 0.0]))
        self.x.configure("pick_x", "soft", self.X_W)

    def solve(self, seed: dict[str, float], p: np.ndarray, inward: np.ndarray, close_dir: np.ndarray,
              iters: int = 150) -> tuple[dict[str, float], float]:
        m, kin = self.model, self.model.kin
        kin._set(m.to_deg(seed))
        kin.robot.update_kinematics()
        self.pos.target_world = np.asarray(p, float)
        z = np.asarray(inward, float) / np.linalg.norm(inward)
        x = np.asarray(close_dir, float) - z * float(np.asarray(close_dir, float) @ z)  # jaws ⊥ approach axis
        if np.linalg.norm(x) < 1e-6:
            raise PathError("the grip direction is parallel to the approach axis")
        self.z.targetAxis_world = z
        self.x.targetAxis_world = x / np.linalg.norm(x)
        for _ in range(iters):
            try:
                self.solver.solve(True)
            except RuntimeError:  # hard position infeasible from here (out of reach)
                break
            kin.robot.update_kinematics()
        pose = m.to_units(np.rad2deg([kin.robot.get_joint(j) for j in kin.joints]))
        return pose, float(np.linalg.norm(m.tip(pose) - p))


def best_targets(model: ArmModel, ik: AxisIK, p: np.ndarray, inward: np.ndarray, jaw_x: np.ndarray,
                 seeds: list[dict[str, float]]) -> tuple[list[np.ndarray], list[dict[str, float]]]:
    """Achievable tool targets at p with tool x along `jaw_x` (toward the fixed jaw), best first, plus the joint
    poses that reach them (use as IK seeds)."""
    sols = []
    for d in (jaw_x,):
        for seed in seeds:
            q, err = ik.solve(seed, p, inward, d)
            over = any(abs(v) > JOINT_LIMIT for v in q.values())
            # tie-break: wrist roll nearest the middle of its range (margin both ways)
            sols.append(((over, err > MAX_IK_ERR_M, round(err * 1000), round(abs(q["wrist_roll"]))), q))
    sols.sort(key=lambda r: r[0])
    targets, poses = [], []
    for _, q in sols[:4]:
        T = model.fk(q)
        T[:3, 3] = p
        targets.append(T)
        poses.append(q)
    return targets, poses


def pitched(inward: np.ndarray, pitch_deg: float) -> np.ndarray:
    """`inward` turned `pitch_deg` toward straight down (world -z)."""
    if pitch_deg == 0:
        return inward
    axis = np.cross(inward, [0.0, 0.0, -1.0])
    if np.linalg.norm(axis) < 1e-6:
        return inward  # already pointing straight down
    return Rotation.from_rotvec(axis / np.linalg.norm(axis) * math.radians(pitch_deg)).apply(inward)


def _seeds(current: dict[str, float]) -> list[dict[str, float]]:
    bent = dict(shoulder_lift=-10, elbow_flex=10, wrist_flex=90)
    return [{"shoulder_pan": float(pan), "wrist_roll": float(roll), **bent}
            for pan in (current["shoulder_pan"], -30, 0, 30) for roll in (0, 50, -50)]


def plan_pick(model: ArmModel, pocket: str, sku: str, point_m: np.ndarray, normal: np.ndarray,
              close_dir: np.ndarray, current: dict[str, float], s: PickSettings) -> PickPlan:
    """Plan the pick program from joint pose `current`. Raises PathError if no pitch up to `max_pitch_deg`
    fits even the minimum pull-out."""
    n = np.asarray(normal, float) / np.linalg.norm(normal)
    c = np.asarray(point_m, float)
    close_dir = np.asarray(close_dir, float)
    pitches = np.arange(0.0, s.max_pitch_deg + 1e-9, s.pitch_step_deg)
    pulls = sorted({s.pull_mm, s.extract_mm}, reverse=True)
    errors: list[PathError] = []
    for pull in pulls:
        for pitch in pitches:
            # the fixed jaw can be on either side of the part (wrist roll 180° apart): plan both, keep the one
            # whose wrist roll is nearest mid-range — unless the type pins the side (close neighbour on the other)
            plans = []
            sides = {None: (close_dir, -close_dir), "+": (close_dir,), "-": (-close_dir,)}[s.fixed_jaw_side]
            for jaw_x in sides:
                try:
                    plans.append(_plan_one(model, pocket, sku, c, n, jaw_x, current, s, float(pitch), pull))
                except PathError as e:
                    errors.append(e.wrap(f"pitch {pitch:.0f}°/pull {pull:.0f} mm: {e}"))
            if plans:
                return min(plans, key=lambda pl: abs(pl.steps[1].pose["wrist_roll"]))
    raise errors[-1].wrap(f"no tool pitch up to {s.max_pitch_deg:.0f}° fits a {pulls[-1]:.0f} mm pull-out from "
                          f"{pocket}. Last: {errors[-1]}")


def _plan_one(model, pocket, sku, c, n, jaw_x, current, s, pitch, pull) -> PickPlan:
    """One candidate: tool x (toward the fixed jaw) along `jaw_x`. The tool point is aimed beside the part:
    shifted toward the fixed jaw (in the tray plane) so the fixed jaw clears the part and the part sits centered
    in the opening."""
    side = jaw_x - n * float(jaw_x @ n)
    side /= np.linalg.norm(side)
    aim = c + side * s.fixed_jaw_offset_mm / 1000
    approach = aim + n * (s.protrusion_mm + s.approach_mm) / 1000
    grasp = aim + n * s.grasp_standoff_mm / 1000
    close_dir = jaw_x
    inward = pitched(-n, pitch)
    b = ProgramBuilder(model, current)
    seeds = _seeds(current)
    # cheap reject first: the far end of the pull is the hardest point to reach
    far = aim + n * (s.grasp_standoff_mm + pull) / 1000
    ik = AxisIK(model)
    targets, poses = best_targets(model, ik, far, inward, close_dir, seeds)
    ProgramBuilder(model, current).joint_to(targets[0], "pull-out end", poses, targets=targets[:1])

    b.open_gripper(f"open to {s.open_value} (part {s.grip_mm:g} mm + margin)", s.open_value)
    targets, poses = best_targets(model, ik, approach, inward, close_dir, seeds)
    move = b.joint_to(targets[0], "approach point", poses, targets=targets)
    travel = max(abs(move.pose[j] - b.start[j]) for j in move.pose)
    line_in = b.line_to(grasp, IN_SPEED_MM_S, f"in to {s.grasp_standoff_mm:.1f} mm off {pocket}")
    back_out = LineMove(list(reversed(line_in.path)), line_in.length_mm, IN_SPEED_MM_S, "back out (missed)")
    b.grip(s.min_grip, f"grip {sku}", on_fail=[OpenGripper("open (missed)", s.open_value), back_out])
    b.line_by(n * pull / 1000, PULL_SPEED_MM_S, f"pull out {pull:.0f} mm")
    b.check_holding(s.min_grip)
    lift = 0.0
    for frac in (1.0, 2 / 3, 1 / 3):
        m = b.mark()
        try:
            b.line_by(np.array([0.0, 0.0, s.lift_mm * frac / 1000]), LIFT_SPEED_MM_S, f"lift {s.lift_mm * frac:.0f} mm")
            lift = s.lift_mm * frac
            break
        except PathError:
            b.reset(m)
    if not lift:
        raise PathError(f"can't lift even {s.lift_mm / 3:.0f} mm after the pull-out", b.tip, list(b.steps), b.start)
    b.check_holding(s.min_grip, "check holding after lift")
    R_grasp = model.fk(line_in.path[-1])[:3, :3]
    tilt = _angle(R_grasp[:, 2], -n)
    jaw_off = _angle(R_grasp[:, 0], jaw_x)
    pull_path = next(st for st in b.steps if isinstance(st, LineMove) and st.label.startswith("pull")).path
    twist = max(math.degrees(Rotation.from_matrix(R_grasp.T @ model.fk(q)[:3, :3]).magnitude()) for q in pull_path)
    return PickPlan(pocket, sku, [round(float(v), 5) for v in c], b.steps, pitch, tilt, pull, lift,
                    s.grasp_standoff_mm, travel, twist, jaw_off, s.grip_mm, s.open_value, s.fixed_jaw_offset_mm)


def _angle(a: np.ndarray, b: np.ndarray) -> float:
    a, b = np.asarray(a, float), np.asarray(b, float)
    return math.degrees(math.acos(max(-1.0, min(1.0, float(a @ b / np.linalg.norm(a) / np.linalg.norm(b))))))


def plan_pocket_pick(model: ArmModel, tray_cfg, gripper_cfg, cal, pocket_id: str,
                     current: dict[str, float]) -> PickPlan:
    """plan_pick for a pocket id, everything taken from the tray config + calibration `cal`."""
    layout = tray_cfg.layout()
    pk = layout.pocket(pocket_id)
    s = PickSettings.for_type(tray_cfg, pk.sku, gripper_cfg)
    close_dir = cal.dir_to_robot(*pk.grip_dir(s.grip_axis))
    return plan_pick(model, pk.id, pk.sku, cal.to_robot(pk.x, pk.y), cal.normal, close_dir, current, s)
