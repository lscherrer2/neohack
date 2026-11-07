"""High-level, conservative jog commands for an LLM (or a human) driving the arm step by step.

The caller thinks in directions ("forward 2 cm", "tilt down 10 degrees"), never joints. Every
command is planned in full and checked against `RemoteLimits` before the arm moves; if any check
fails nothing moves and the reason is returned. Moves are straight lines at slow speed, with the
normal Motion safety stops (following error, settle).
"""

from __future__ import annotations

import json
import math
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path

import numpy as np

from ..joints import BODY
from ..motion.controller import Motion, MotionError
from ..motion.kinematics import ArmModel, PathError
from .safety import RemoteLimits

# Robot base frame, as seen standing behind the robot looking where it faces.
DIRECTIONS = {
    "forward": (1.0, 0.0, 0.0),
    "back": (-1.0, 0.0, 0.0),
    "left": (0.0, 1.0, 0.0),
    "right": (0.0, -1.0, 0.0),
    "up": (0.0, 0.0, 1.0),
    "down": (0.0, 0.0, -1.0),
}
TILTS = {"down": 1.0, "up": -1.0}  # gripper nose down / up (rotation about the arm's sideways axis)
TWISTS = {"left": 1.0, "right": -1.0}  # wrist roll; sign follows the wrist_roll joint


@dataclass
class JogResult:
    ok: bool
    message: str
    tip_cm: list[float] = field(default_factory=list)  # x forward, y left, z up
    gripper: float | None = None

    def text(self) -> str:
        status = "OK" if self.ok else "REFUSED/STOPPED"
        tip = ", ".join(f"{a}={v:+.1f}" for a, v in zip(("x fwd", "y left", "z up"), self.tip_cm))
        grip = "" if self.gripper is None else f"; gripper {self.gripper:.0f} (0..100)"
        return f"{status}: {self.message}\nTip position (cm): {tip}{grip}"


def _explain(error: PathError) -> str:
    """Plain-language version of a path planning failure, with a hint for what to try."""
    text = str(error)
    if "joint jump" in text or "unreachable" in text:
        return ("the arm can't do that smoothly from here (nearly stretched out or an awkward angle). "
                "Try a smaller step, or move back/up first.")
    if "joint limit" in text:
        return "a joint would hit its limit. Try a different direction, or go home and approach differently."
    return text


def _rot_about(axis: np.ndarray, angle: float) -> np.ndarray:
    """Rotation matrix for `angle` radians about unit `axis` (Rodrigues)."""
    x, y, z = axis
    k = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + math.sin(angle) * k + (1 - math.cos(angle)) * (k @ k)


class Jogger:
    """Turns direction commands into checked, slow, straight-line arm motion."""

    def __init__(self, motion: Motion, model: ArmModel, limits: RemoteLimits, log_path: Path | None = None):
        self.motion, self.model, self.limits = motion, model, limits
        self.log_path = log_path
        roll = BODY.index("wrist_roll")
        self.twist_units_per_deg = 1.0 / float(model.scale[roll])  # model.scale = degrees per unit

    # ---------- state ----------
    def _pose(self) -> dict[str, float]:
        return {j: self.motion.cmd[j] for j in BODY}

    def _result(self, ok: bool, message: str) -> JogResult:
        tip = self.model.tip(self._pose()) * 100
        result = JogResult(ok, message, [round(float(v), 1) for v in tip], round(self.motion.cmd["gripper"], 1))
        self._log(result)
        return result

    def _log(self, result: JogResult) -> None:
        if self.log_path is None:
            return
        self.log_path.parent.mkdir(parents=True, exist_ok=True)
        entry = {"ts": time.strftime("%Y-%m-%dT%H:%M:%S"), **asdict(result)}
        with open(self.log_path, "a") as f:
            f.write(json.dumps(entry) + "\n")

    def status(self) -> JogResult:
        actual = self.motion.read()
        joints = ", ".join(f"{j}={actual[j]:.0f}" for j in BODY)
        return self._result(True, f"holding. Joints (-100..100): {joints}")

    # ---------- commands ----------
    def move(self, direction: str, cm: float | None = None) -> JogResult:
        direction = direction.strip().lower()
        if direction not in DIRECTIONS:
            return self._result(False, f"unknown direction '{direction}'. Use one of {list(DIRECTIONS)}")
        cm = self.limits.default_step_cm if cm is None else float(cm)
        if not 0 < cm <= self.limits.max_step_cm:
            return self._result(False, f"distance must be between 0 and {self.limits.max_step_cm} cm")
        start = self._pose()
        goal = self.model.tip(start) + np.array(DIRECTIONS[direction]) * cm / 100
        return self._run_line(start, goal, None, f"moved {direction} {cm:g} cm",
                              duration=cm * 10 / self.limits.tip_speed_mm_s)

    def tilt(self, direction: str, degrees: float | None = None) -> JogResult:
        direction = direction.strip().lower()
        if direction not in TILTS:
            return self._result(False, f"unknown tilt '{direction}'. Use 'up' or 'down'")
        ok, degrees = self._check_angle(degrees)
        if not ok:
            return self._result(False, degrees)
        start = self._pose()
        T = self.model.fk(start)
        local = self.model.local_rot(T)
        # pre-multiplying in the arm's local frame: +angle about local y turns the tool (local z,
        # pointing out of the gripper) toward -z, i.e. nose down
        new_local = _rot_about(np.array([0.0, 1.0, 0.0]), math.radians(TILTS[direction] * degrees)) @ local
        return self._run_line(start, T[:3, 3], new_local, f"tilted {direction} {degrees:g} degrees",
                              duration=degrees / self.limits.turn_speed_deg_s, orientation_weight=1.0)

    def twist(self, direction: str, degrees: float | None = None) -> JogResult:
        direction = direction.strip().lower()
        if direction not in TWISTS:
            return self._result(False, f"unknown twist '{direction}'. Use 'left' or 'right'")
        ok, degrees = self._check_angle(degrees)
        if not ok:
            return self._result(False, degrees)
        target = self._pose()
        target["wrist_roll"] += TWISTS[direction] * degrees * self.twist_units_per_deg
        if reason := self.limits.check_joints(target):
            return self._result(False, f"refused, nothing moved: {reason}")
        try:
            self.motion.move_to(target, speed=self.limits.turn_speed_deg_s * self.twist_units_per_deg)
        except MotionError as e:
            return self._result(False, f"STOPPED during twist: {e}")
        return self._result(True, f"twisted {direction} {degrees:g} degrees")

    def gripper(self, action: str) -> JogResult:
        action = action.strip().lower()
        try:
            if action == "open":
                self.motion.open_gripper()
                return self._result(True, "gripper opened")
            if action == "close":
                width = self.motion.grip(min_width=0.0)
                held = width >= self.motion.gripper.min_part_width
                what = f"something is held (gap {width:.0f})" if held else "closed on nothing"
                return self._result(True, f"gripper closed: {what}")
        except MotionError as e:
            return self._result(False, f"STOPPED: {e}")
        return self._result(False, "action must be 'open' or 'close'")

    def home(self) -> JogResult:
        """Joint-space move back to the calibration mid pose (slow)."""
        try:
            self.motion.move_to("home", speed=15.0)
        except MotionError as e:
            return self._result(False, f"STOPPED on the way home: {e}")
        return self._result(True, "at home pose")

    # ---------- helpers ----------
    def _check_angle(self, degrees: float | None) -> tuple[bool, float | str]:
        degrees = self.limits.default_angle_deg if degrees is None else float(degrees)
        if not 0 < degrees <= self.limits.max_angle_deg:
            return False, f"angle must be between 0 and {self.limits.max_angle_deg} degrees"
        return True, degrees

    def _run_line(self, start: dict, goal_xyz: np.ndarray, end_local_rot: np.ndarray | None, done: str,
                  duration: float, orientation_weight: float = 0.05) -> JogResult:
        limits, pan_axis = self.limits, self.model.pan_xy
        if reason := limits.check_tip(goal_xyz, pan_axis):
            return self._result(False, f"refused, nothing moved: {reason}")
        try:
            path = self.model.line(start, goal_xyz, end_local_rot=end_local_rot,
                                   orientation_weight=orientation_weight, limit=limits.joint_limit)
        except PathError as e:
            return self._result(False, f"refused, nothing moved: {_explain(e)}")
        for pose in path:  # every intermediate point, not just the end
            if reason := limits.check_tip(self.model.tip(pose), pan_axis):
                return self._result(False, f"refused, nothing moved: path leaves the safe zone ({reason})")
        try:
            self.motion.follow(path, max(0.5, duration), done)
        except MotionError as e:
            return self._result(False, f"STOPPED mid-move: {e}")
        return self._result(True, done)
