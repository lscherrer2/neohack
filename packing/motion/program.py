"""Motion programs: a procedure as a list of steps (nouns + verbs), planned up front, then executed.

G-code-like, but with no modal state: every step says what it means (`line_to` a point vs `line_by` an offset,
its own speed), so a step can't silently change meaning because of an earlier "relative mode" line.

Two layers:
  * `ProgramBuilder` (plan time, no hardware): verbs with geometric arguments — `joint_to(T)`, `line_to(p)`,
    `line_by(d)`, `open_gripper()`, `grip(min_width)`, `check_holding(min_width)`. It tracks the planned pose
    and turns each verb into a concrete, validated step (IK solved, reach / joint limits / jumps checked), so a
    program that builds is a program whose every motion was checked before anything moves.
  * `run(steps, motion)` (run time): plays the steps on a `Motion` (which adds the safety checks, feed-forward,
    following-error stop). A Grip may carry `on_fail` steps (e.g. a missed grip backs out) — they run on a
    GraspError, then the error is re-raised.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable, Union

import numpy as np

from ..joints import BODY
from .kinematics import ArmModel, PathError

Pose = dict[str, float]
JOINT_LIMIT = 100.0
MAX_IK_ERR_M = 0.002


# ---------- steps (the executable nouns) ----------
@dataclass
class JointMove:
    """Free joint-space move (curved tip path) to `pose`; `speed` in units/s (None = configured speed)."""

    pose: Pose
    speed: float | None = None
    label: str = "joint move"


@dataclass
class LineMove:
    """Straight tip line, already planned as dense joint poses; played at `speed_mm_s`."""

    path: list[Pose]
    length_mm: float
    speed_mm_s: float
    label: str = "line"

    @property
    def duration(self) -> float:
        return max(1.0, self.length_mm / self.speed_mm_s)


@dataclass
class OpenGripper:
    label: str = "open gripper"
    value: float | None = None  # gripper value; None = fully open (gripper.open)


@dataclass
class Grip:
    """Close until the fingers stall; GraspError if narrower than `min_width` (nothing grasped)."""

    min_width: float
    label: str = "grip"
    on_fail: list = field(default_factory=list)
    measured: float | None = None  # filled in by run(): grip width (gripper units from closed)


@dataclass
class CheckHolding:
    """Still holding something? GraspError if the gripper has closed past `min_width` (part dropped)."""

    min_width: float
    label: str = "check holding"


Step = Union[JointMove, LineMove, OpenGripper, Grip, CheckHolding]


def describe(steps: list[Step]) -> list[dict]:
    """Short JSON-able listing (dry runs, confirm dialogs, logs)."""
    out = []
    for s in steps:
        d: dict = {"step": type(s).__name__, "label": s.label}
        if isinstance(s, LineMove):
            d.update(length_mm=round(s.length_mm, 1), speed_mm_s=s.speed_mm_s)
        elif isinstance(s, (Grip, CheckHolding)):
            d["min_width"] = s.min_width
        elif isinstance(s, OpenGripper) and s.value is not None:
            d["value"] = round(s.value, 1)
        out.append(d)
    return out


# ---------- plan time ----------
class ProgramBuilder:
    """Builds a validated step list from geometric verbs, starting at joint pose `start`."""

    def __init__(self, model: ArmModel, start: Pose):
        self.model = model
        self.start = {j: float(start[j]) for j in BODY}
        self.pose = dict(self.start)  # planned body pose after the last step
        self.steps: list[Step] = []

    @property
    def tip(self) -> np.ndarray:
        return self.model.tip(self.pose)

    def joint_to(self, T: np.ndarray, label: str, seeds: list[Pose] | None = None, speed: float | None = None,
                 targets: list[np.ndarray] | None = None) -> JointMove:
        """Free move to a 4x4 tool target. `targets`: alternative equivalent targets (e.g. the gripper turned
        180° about its axis) — the best in-limit solution over all targets and seeds wins."""
        best = None
        for Ti in targets or [T]:
            for seed in [self.pose, *(seeds or [])]:
                q, err = self.model.ik({j: seed[j] for j in BODY}, Ti, orientation_weight=0.5, iters=150)
                over = any(abs(v) > JOINT_LIMIT for v in q.values())
                score = (over, err > MAX_IK_ERR_M, err if err > MAX_IK_ERR_M else 0.0,
                         max(abs(q[j] - self.pose[j]) for j in BODY))
                if best is None or score < best[0]:
                    best = (score, q, err, Ti)
        (over, _, _, _), q, err, Ti = best
        where = f"{label} ({np.round(Ti[:3, 3] * 1000, 1)} mm)"
        if err > MAX_IK_ERR_M:
            raise PathError(f"can't reach the {where}: IK off by {err * 1000:.1f} mm")
        if over:
            raise PathError(f"joint limit at the {where}: {({j: round(v, 1) for j, v in q.items() if abs(v) > JOINT_LIMIT})}")
        step = JointMove(q, speed, label)
        self.steps.append(step)
        self.pose = q
        return step

    def line_to(self, p_m: np.ndarray, speed_mm_s: float, label: str) -> LineMove:
        """Straight line from the current tip to point `p_m` (meters), tool orientation held."""
        start_tip = self.tip
        try:
            path = self.model.line(self.pose, np.asarray(p_m, float), limit=JOINT_LIMIT)
        except PathError as e:
            raise PathError(f"{label}: {e}") from e
        step = LineMove(path, float(np.linalg.norm(np.asarray(p_m) - start_tip) * 1000), speed_mm_s, label)
        self.steps.append(step)
        self.pose = path[-1]
        return step

    def line_by(self, d_m: np.ndarray, speed_mm_s: float, label: str) -> LineMove:
        """Straight line by offset `d_m` (meters, robot frame) from the current tip."""
        return self.line_to(self.tip + np.asarray(d_m, float), speed_mm_s, label)

    def open_gripper(self, label: str = "open gripper", value: float | None = None) -> OpenGripper:
        step = OpenGripper(label, value)
        self.steps.append(step)
        return step

    def grip(self, min_width: float, label: str = "grip", on_fail: list[Step] | None = None) -> Grip:
        step = Grip(min_width, label, list(on_fail or []))
        self.steps.append(step)
        return step

    def check_holding(self, min_width: float, label: str = "check holding") -> CheckHolding:
        step = CheckHolding(min_width, label)
        self.steps.append(step)
        return step

    def mark(self) -> tuple[int, Pose]:
        """Remember the build state (to try alternatives and roll back with `reset`)."""
        return len(self.steps), dict(self.pose)

    def reset(self, mark: tuple[int, Pose]) -> None:
        n, pose = mark
        del self.steps[n:]
        self.pose = pose


# ---------- run time ----------
def run(steps: list[Step], motion, on_step: Callable[[Step], None] | None = None) -> None:
    """Execute `steps` on a Motion. A step that fails with a GraspError and has `on_fail` runs those steps, then
    re-raises. Any other error (Stop, following error, bus) is raised at once — no recovery motion."""
    from .controller import GraspError

    for step in steps:
        if on_step:
            on_step(step)
        try:
            _run_one(step, motion, GraspError)
        except GraspError:
            if getattr(step, "on_fail", None):
                run(step.on_fail, motion, on_step)
            raise


def _run_one(step: Step, motion, GraspError) -> None:
    if isinstance(step, JointMove):
        motion.move_to(step.pose, speed=step.speed)
    elif isinstance(step, LineMove):
        motion.follow(step.path, step.duration, step.label)
    elif isinstance(step, OpenGripper):
        motion.open_gripper(step.value)
    elif isinstance(step, Grip):
        step.measured = motion.grip(step.min_width)
    elif isinstance(step, CheckHolding):
        width = abs(motion.read()["gripper"] - motion.gripper.closed)
        if width < step.min_width:
            raise GraspError(f"gripper closed to width {width:.1f} (< {step.min_width}) — part dropped")
    else:
        raise TypeError(f"unknown step {step!r}")
