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
    """The physical SO-101 follower, via LeRobot's SO101Follower. Connects (torque on) on creation."""

    def __init__(self, port: str, robot_id: str, max_step: float = 15.0):
        from lerobot.robots.so101_follower import SO101Follower, SO101FollowerConfig

        config = SO101FollowerConfig(port=port, id=robot_id, max_relative_target=float(max_step))
        self.robot = SO101Follower(config)
        if not self.robot.calibration:
            raise RuntimeError(f"No calibration file for '{robot_id}'. Calibrate the arm first.")
        self.robot.connect(calibrate=False)  # enables torque, holds current pose

    def tune(self, p_gain: int | None = None, accel: int | None = None) -> None:
        """Servo control tuning for the body joints. Torque is on => EEPROM is locked, so these land in
        RAM only: they last until power-off / next connect (LeRobot's configure() resets P to 16)."""
        for joint in BODY:
            if p_gain is not None:
                self.robot.bus.write("P_Coefficient", joint, int(p_gain))
            if accel is not None:
                self.robot.bus.write("Acceleration", joint, int(accel))

    def read(self) -> dict[str, float]:
        positions = self.robot.bus.sync_read("Present_Position")
        return {joint: float(value) for joint, value in positions.items()}

    def write(self, pose: dict[str, float]) -> None:
        self.robot.send_action({f"{joint}.pos": float(pose[joint]) for joint in JOINTS})

    def set_torque(self, on: bool) -> None:
        if on:
            self.robot.bus.enable_torque()
        else:
            self.robot.bus.disable_torque()

    def close(self, release: bool = False) -> None:
        """release=True turns torque off (arm goes limp)."""
        if release:
            self.robot.disconnect()
        else:
            self.robot.bus.disconnect(disable_torque=False)


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
