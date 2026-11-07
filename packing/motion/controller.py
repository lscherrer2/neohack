"""Safe, smooth joint-space motion and gripper control on top of an `Arm`.

Every command goes through `Motion._tick`, which writes the command, reads the arm back, learns the
gravity-sag correction, checks the following error and paces the loop. All moves and the gripper
share it, so the safety checks apply everywhere.
"""

from __future__ import annotations

import math
import time
from typing import Callable

from ..config import GripperCfg, MotionCfg
from ..hardware.arm import Arm
from ..joints import BODY, JOINTS, clamp_joint
from ..poses import PoseStore

Pose = dict[str, float]

GRIPPER_OPEN_TOL = 4.0  # units: how close the gripper must get to `open` to count as open
STALL_MOVE_EPS = 0.3  # gripper moved less than this per tick = standing still
STALL_TICKS = 8  # standing still this many ticks while still being driven = contact
STALL_MIN_DRIVE = 2.0  # ...and the command must be at least this far past the gripper
CLOSE_SETTLE_TICKS = 10  # ticks to let the gripper finish closing before measuring the width


class MotionError(RuntimeError):
    """Arm did not get where it was told (blocked, collision, unreachable)."""


class GraspError(MotionError):
    """Gripper closed on nothing / object too small: nothing was picked."""


def _smoothstep(a: float) -> float:
    """Ease 0..1 with zero slope at both ends (gentle start and stop)."""
    return a * a * (3 - 2 * a)


class Motion:
    """Moves the arm between poses with speed limits and safety stops."""

    def __init__(self, arm: Arm, poses: PoseStore, cfg: MotionCfg, gripper: GripperCfg,
                 sleep: Callable[[float], None] = time.sleep, log: Callable[[str], None] = print):
        self.arm, self.poses, self.cfg, self.gripper = arm, poses, cfg, gripper
        self.sleep, self.log = sleep, log
        self.tick_period = 1.0 / cfg.fps
        self._next_tick_time = time.monotonic()
        self.closing_direction = 1.0 if gripper.closed >= gripper.open else -1.0
        self.trace: list | None = None  # set to [] to record (t, cmd, actual) every tick
        self.cmd = arm.read()  # last TARGET pose (all joints); what's written also includes `bias`
        # Sag compensation: an outer integral loop. The servos are proportional-only, so a loaded joint
        # (mainly shoulder_lift) settles short of its target. bias slowly learns that error and is added
        # to the command, so the joint actually arrives. Clamped to +-sag_max units.
        self.bias = dict.fromkeys(BODY, 0.0)

    # ---------- basics ----------
    def read(self) -> Pose:
        return self.arm.read()

    def hold(self) -> None:
        """Freeze where the arm actually is (used on errors / Ctrl+C)."""
        self.cmd = self.read()
        self.bias = dict.fromkeys(BODY, 0.0)
        self.arm.write(self.cmd)

    def _resolve(self, target: str | Pose) -> Pose:
        return self.poses.get(target) if isinstance(target, str) else dict(target)

    # ---------- the control tick ----------
    def _tick(self, cmd: Pose, check_joints: list[str]) -> Pose:
        """Send `cmd`, return the arm's actual pose. Raises MotionError (after holding) if any of
        `check_joints` lags `cmd` by more than max_following_error."""
        self.arm.write(self._with_bias(cmd))
        self.cmd = cmd
        actual = self.read()
        self._learn_sag(cmd, actual)
        if self.trace is not None:
            self.trace.append((time.monotonic(), dict(cmd), dict(actual)))
        self._check_following_error(cmd, actual, check_joints)
        self._pace()
        return actual

    def _with_bias(self, cmd: Pose) -> Pose:
        out = dict(cmd)
        for joint in BODY:
            out[joint] = clamp_joint(joint, cmd[joint] + self.bias[joint])
        return out

    def _learn_sag(self, cmd: Pose, actual: Pose) -> None:
        gain, limit = self.cfg.sag_gain, self.cfg.sag_max
        if gain <= 0:
            return
        for joint in BODY:
            learned = self.bias[joint] + gain * (cmd[joint] - actual[joint])
            self.bias[joint] = max(-limit, min(limit, learned))

    def _check_following_error(self, cmd: Pose, actual: Pose, check_joints: list[str]) -> None:
        error = {joint: abs(cmd[joint] - actual[joint]) for joint in check_joints}
        worst = max(error, key=error.get)
        if error[worst] > self.cfg.max_following_error:
            self.hold()
            raise MotionError(f"{worst} lagging by {error[worst]:.1f} (blocked or collision?) — stopped")

    def _pace(self) -> None:
        """Deadline-based pacing: bus time doesn't stretch the move."""
        now = time.monotonic()
        self._next_tick_time = max(self._next_tick_time + self.tick_period, now)
        self.sleep(max(0.0, self._next_tick_time - now))

    def _settle(self, target: Pose, joints: list[str], tol: float, what: str) -> None:
        """Keep commanding `target` until every one of `joints` is within `tol`, or time out."""
        self._next_tick_time = time.monotonic()
        for _ in range(max(1, int(self.cfg.settle_timeout * self.cfg.fps))):
            actual = self._tick({**self.cmd, **{j: target[j] for j in joints}}, BODY)  # keeps correcting
            if all(abs(target[j] - actual[j]) <= tol for j in joints):
                return
        off = {j: round(target[j] - actual[j], 1) for j in joints if abs(target[j] - actual[j]) > tol}
        self.hold()
        raise MotionError(f"did not reach {what}: off by {off}")

    # ---------- moves ----------
    def move_to(self, target: str | Pose, speed: float | None = None) -> None:
        """Interpolated joint-space move of the 5 body joints. Gripper keeps its current command."""
        name = target if isinstance(target, str) else "pose"
        goal_body = self._resolve(target)
        speed = speed or self.cfg.speed
        start = dict(self.cmd)
        goal = {**start, **{j: goal_body[j] for j in BODY}}
        distance = max(abs(goal[j] - start[j]) for j in BODY)
        steps = max(1, math.ceil(distance / speed * self.cfg.fps))
        self._next_tick_time = time.monotonic()
        for i in range(1, steps + 1):
            progress = _smoothstep(i / steps)
            self._tick({j: start[j] + (goal[j] - start[j]) * progress for j in JOINTS}, BODY)
        self._settle(goal, BODY, self.cfg.settle_tol, name)

    def follow(self, path: list[Pose], duration: float, name: str = "path") -> None:
        """Play a dense list of body poses (e.g. from ArmModel.line) over `duration` seconds with a
        smooth start/stop. Same safety checks as move_to."""
        if len(path) < 2:
            return
        start_error = max(abs(path[0][j] - self.cmd[j]) for j in BODY)
        if start_error > self.cfg.settle_tol:
            raise MotionError(f"{name} starts {start_error:.1f} units away from the arm — move there first")
        steps = max(1, math.ceil(duration * self.cfg.fps))
        self._next_tick_time = time.monotonic()
        last = len(path) - 1
        for i in range(1, steps + 1):
            position = _smoothstep(i / steps) * last  # fractional index along the path
            k = min(int(position), last - 1)
            frac = position - k
            body = {j: path[k][j] + (path[k + 1][j] - path[k][j]) * frac for j in BODY}
            self._tick({**self.cmd, **body}, BODY)
        self._settle({**self.cmd, **path[-1]}, BODY, self.cfg.settle_tol, name)

    # ---------- gripper ----------
    def _gripper_to(self, value: float) -> None:
        start = self.cmd["gripper"]
        steps = max(1, math.ceil(abs(value - start) / self.gripper.speed * self.cfg.fps))
        for i in range(1, steps + 1):
            self._tick({**self.cmd, "gripper": start + (value - start) * i / steps}, BODY)

    def open_gripper(self) -> None:
        self._gripper_to(self.gripper.open)
        self._settle({**self.cmd, "gripper": self.gripper.open}, ["gripper"], GRIPPER_OPEN_TOL, "gripper open")

    def grip(self, min_width: float) -> float:
        """Close until the fingers stall on an object. Returns grip width (distance from `closed`).
        Raises GraspError if it closed (almost) fully, i.e. nothing in the gripper."""
        closed, direction = self.gripper.closed, self.closing_direction
        step = self.gripper.speed / self.cfg.fps
        command = self.cmd["gripper"]
        last_position, still_ticks = self.read()["gripper"], 0
        while (closed - command) * direction > 0:
            command += direction * min(step, abs(closed - command))
            position = self._tick({**self.cmd, "gripper": command}, BODY)["gripper"]
            still_ticks = still_ticks + 1 if abs(position - last_position) < STALL_MOVE_EPS else 0
            last_position = position
            if still_ticks >= STALL_TICKS and abs(command - position) > STALL_MIN_DRIVE:
                break  # commanded past the gripper but it isn't moving: contact
        for _ in range(CLOSE_SETTLE_TICKS):
            position = self._tick({**self.cmd, "gripper": command}, BODY)["gripper"]
        width = abs(position - closed)
        if width < min_width:
            raise GraspError(f"gripper closed to {position:.1f} (width {width:.1f} < {min_width}) — nothing grasped")
        # hold slightly past contact so the grip stays firm but not stalled hard
        self._tick({**self.cmd, "gripper": position + direction * self.gripper.squeeze}, BODY)
        return width
