"""Arm I/O: `RealArm` talks to the SO-101 follower, `FakeArm` simulates it for tests and `--sim`.

Both implement the `Arm` protocol, so everything above this layer is hardware-agnostic.
"""

from __future__ import annotations

from typing import Protocol

from ..joints import BODY, JOINTS


class Arm(Protocol):
    """What the motion layer needs from an arm. Poses map joint name -> calibrated units."""

    def read(self) -> dict[str, float]:
        """Current position of every joint."""

    def write(self, pose: dict[str, float]) -> None:
        """Command a target for every joint."""

    def set_torque(self, on: bool) -> None: ...

    def tune(self, p_gain: int | None = None, accel: int | None = None) -> None:
        """Set servo P gain / acceleration on the body joints (None = leave unchanged)."""

    def close(self, release: bool = False) -> None:
        """Disconnect. release=True also turns torque off (arm goes limp)."""


class RealArm:
    """The physical SO-101 follower, via our Feetech driver (`so101.SO101`). Connects (torque on) on creation."""

    def __init__(self, port: str, robot_id: str, max_step: float = 15.0):
        from .so101 import SO101

        self.robot = SO101(port, robot_id, max_relative_target=float(max_step))  # configures, holds current pose

    def tune(self, p_gain: int | None = None, accel: int | None = None) -> None:
        """Servo control tuning for the body joints. Torque is on => EEPROM is locked, so these land in
        RAM only: they last until power-off / next connect (connect resets P to 16, accel to 254)."""
        if p_gain is not None:
            self.robot.write_register("P_Coefficient", BODY, int(p_gain))
        if accel is not None:
            self.robot.write_register("Acceleration", BODY, int(accel))

    def read(self) -> dict[str, float]:
        return self.robot.read()

    def write(self, pose: dict[str, float]) -> None:
        self.robot.write({joint: float(pose[joint]) for joint in JOINTS})

    def set_torque(self, on: bool) -> None:
        self.robot.set_torque(on)

    def close(self, release: bool = False) -> None:
        """release=True turns torque off (arm goes limp)."""
        self.robot.close(release=release)


class FakeArm:
    """Simulated arm. Each read() moves every joint `gain` of the way to its target.

    `gripper_block` simulates an object in the gripper: the gripper cannot pass that value in the
    closing direction. `sag` simulates gravity: a joint settles `sag` units short of its target.
    """

    def __init__(self, start: dict[str, float] | None = None, gain: float = 0.5,
                 closed: float = 0.0, opened: float = 60.0):
        self.pos = dict(start or dict.fromkeys(JOINTS, 0.0))
        self.target = dict(self.pos)
        self.gain = gain
        self.closing_direction = 1.0 if closed >= opened else -1.0
        self.gripper_block: float | None = None
        self.sag = dict.fromkeys(JOINTS, 0.0)
        self.torque = True
        self.writes = 0

    def read(self) -> dict[str, float]:
        for joint in JOINTS:
            reachable = self.target[joint] - self.sag[joint]
            self.pos[joint] += (reachable - self.pos[joint]) * self.gain
        self._apply_gripper_block()
        return dict(self.pos)

    def _apply_gripper_block(self) -> None:
        block = self.gripper_block
        if block is not None and (self.pos["gripper"] - block) * self.closing_direction > 0:
            self.pos["gripper"] = block

    def write(self, pose: dict[str, float]) -> None:
        self.writes += 1
        self.target = {joint: float(pose[joint]) for joint in JOINTS}

    def set_torque(self, on: bool) -> None:
        self.torque = on

    def tune(self, p_gain: int | None = None, accel: int | None = None) -> None:
        """No servos to tune in simulation."""

    def close(self, release: bool = False) -> None:
        self.torque = not release
