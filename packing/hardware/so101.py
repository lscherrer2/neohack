"""SO-101 follower driver straight on the Feetech SDK (`scservo_sdk`), no LeRobot.

Same behavior and units as LeRobot's SO101Follower so taught poses and calibration files carry over:
body joints -100..100 across the calibrated range, gripper 0..100 (0 = closed on our arm).

Calibration lives in two places that must agree: the servos' EEPROM (Homing_Offset, Min/Max_Position_Limit)
and a JSON file {joint: {id, drive_mode, homing_offset, range_min, range_max}} (`calibration_path`).
Write both with `python -m packing.tools.calibrate`.
"""

from __future__ import annotations

import json
import time
from dataclasses import dataclass
from pathlib import Path

from ..joints import JOINTS

PACKING_DIR = Path(__file__).resolve().parent.parent
CALIBRATION_DIR = PACKING_DIR / "calibration"
LEROBOT_CALIBRATION_DIR = Path.home() / ".cache/huggingface/lerobot/calibration/robots/so101_follower"

BAUD = 1_000_000
MOTOR_IDS = {"shoulder_pan": 1, "shoulder_lift": 2, "elbow_flex": 3, "wrist_flex": 4, "wrist_roll": 5, "gripper": 6}

# STS3215 control table: name -> (address, bytes). Addresses < 40 are EEPROM (need Lock=0 to persist).
REG = {
    "Return_Delay_Time": (7, 1),
    "Min_Position_Limit": (9, 2),
    "Max_Position_Limit": (11, 2),
    "Max_Torque_Limit": (16, 2),
    "P_Coefficient": (21, 1),
    "D_Coefficient": (22, 1),
    "I_Coefficient": (23, 1),
    "Protection_Current": (28, 2),
    "Homing_Offset": (31, 2),
    "Operating_Mode": (33, 1),
    "Overload_Torque": (36, 1),
    "Torque_Enable": (40, 1),
    "Acceleration": (41, 1),
    "Goal_Position": (42, 2),
    "Lock": (55, 1),
    "Present_Position": (56, 2),
    "Present_Load": (60, 2),  # sign-magnitude, sign bit 10, 0.1 % of max torque
    "Present_Voltage": (62, 1),
    "Present_Temperature": (63, 1),
    "Maximum_Acceleration": (85, 1),
}
HOMING_SIGN_BIT = 11  # Homing_Offset is sign-magnitude with the sign in bit 11


def encode_sm(value: int, bit: int = HOMING_SIGN_BIT) -> int:
    if abs(value) >= 1 << bit:
        raise ValueError(f"{value} does not fit sign-magnitude bit {bit}")
    return (1 << bit) | -value if value < 0 else value


def decode_sm(raw: int, bit: int = HOMING_SIGN_BIT) -> int:
    return -(raw & ~(1 << bit)) if raw & (1 << bit) else raw


@dataclass
class JointCalibration:
    id: int
    homing_offset: int
    range_min: int
    range_max: int
    drive_mode: int = 0


def calibration_path(robot_id: str) -> Path:
    """Where the calibration for `robot_id` is read from: packing/calibration/, else LeRobot's cache."""
    ours = CALIBRATION_DIR / f"{robot_id}.json"
    legacy = LEROBOT_CALIBRATION_DIR / f"{robot_id}.json"
    return ours if ours.exists() or not legacy.exists() else legacy


def load_calibration(robot_id: str) -> dict[str, JointCalibration]:
    path = calibration_path(robot_id)
    if not path.exists():
        raise RuntimeError(f"no calibration for '{robot_id}' ({path}). Run: python -m packing.tools.calibrate")
    data = json.loads(path.read_text())
    return {j: JointCalibration(**{k: data[j][k] for k in ("id", "homing_offset", "range_min", "range_max", "drive_mode")})
            for j in JOINTS}


def save_calibration(robot_id: str, cal: dict[str, JointCalibration]) -> Path:
    path = CALIBRATION_DIR / f"{robot_id}.json"
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps({j: vars(c) for j, c in cal.items()}, indent=4) + "\n")
    return path


class FeetechBus:
    """Register-level access to the six servos on one serial port."""

    def __init__(self, port: str, baud: int = BAUD, retries: int = 3):
        import scservo_sdk as scs

        self.scs = scs
        self.port = scs.PortHandler(port)
        if not self.port.openPort():
            raise IOError(f"cannot open {port} (another process using it?)")
        self.port.setBaudRate(baud)
        self.packets = scs.PacketHandler(0)
        self.retries = retries

    def close(self) -> None:
        self.port.closePort()

    def ping(self, motor_id: int) -> int | None:
        model, result, _ = self.packets.ping(self.port, motor_id)
        return model if result == self.scs.COMM_SUCCESS else None

    def read(self, reg: str, motor_id: int) -> int:
        addr, size = REG[reg]
        fn = self.packets.read1ByteTxRx if size == 1 else self.packets.read2ByteTxRx
        for _ in range(self.retries):
            value, result, _ = fn(self.port, motor_id, addr)
            if result == self.scs.COMM_SUCCESS:
                return value
            time.sleep(0.005)
        raise IOError(f"read {reg} id {motor_id}: {self.packets.getTxRxResult(result)}")

    def write(self, reg: str, motor_id: int, value: int) -> None:
        addr, size = REG[reg]
        fn = self.packets.write1ByteTxRx if size == 1 else self.packets.write2ByteTxRx
        for _ in range(self.retries):
            result, _ = fn(self.port, motor_id, addr, int(value))
            if result == self.scs.COMM_SUCCESS:
                return
            time.sleep(0.005)
        raise IOError(f"write {reg} id {motor_id}: {self.packets.getTxRxResult(result)}")

    def sync_read(self, reg: str, ids: list[int]) -> dict[int, int]:
        addr, size = REG[reg]
        group = self.scs.GroupSyncRead(self.port, self.packets, addr, size)
        for i in ids:
            group.addParam(i)
        for attempt in range(self.retries):
            if group.txRxPacket() == self.scs.COMM_SUCCESS and all(group.isAvailable(i, addr, size) for i in ids):
                return {i: group.getData(i, addr, size) for i in ids}
            time.sleep(0.005)
        raise IOError(f"sync read {reg} failed for ids {ids}")

    def sync_write(self, reg: str, values: dict[int, int]) -> None:
        addr, size = REG[reg]
        group = self.scs.GroupSyncWrite(self.port, self.packets, addr, size)
        for i, v in values.items():
            v = int(v)
            group.addParam(i, [v & 0xFF] if size == 1 else [v & 0xFF, (v >> 8) & 0xFF])
        if group.txPacket() != self.scs.COMM_SUCCESS:
            raise IOError(f"sync write {reg} failed")


class SO101:
    """The follower arm in calibrated units. Connecting configures the servos (as LeRobot does)
    and turns torque on at the current pose, so nothing jumps."""

    def __init__(self, port: str, robot_id: str, max_relative_target: float | None = None):
        self.cal = load_calibration(robot_id)
        self.ids = {j: c.id for j, c in self.cal.items()}
        self.max_relative_target = max_relative_target
        self.bus = FeetechBus(port)
        try:
            self._check_calibration(robot_id)
            self._configure()
        except Exception:
            self.bus.close()
            raise

    # ---- units ----
    def to_units(self, joint: str, raw: int) -> float:
        c = self.cal[joint]
        raw = min(c.range_max, max(c.range_min, raw))
        f = (raw - c.range_min) / (c.range_max - c.range_min)
        if joint == "gripper":
            return 100 - f * 100 if c.drive_mode else f * 100
        return -(f * 200 - 100) if c.drive_mode else f * 200 - 100

    def to_raw(self, joint: str, value: float) -> int:
        c = self.cal[joint]
        if joint == "gripper":
            v = min(100.0, max(0.0, 100 - value if c.drive_mode else value))
            f = v / 100
        else:
            v = min(100.0, max(-100.0, -value if c.drive_mode else value))
            f = (v + 100) / 200
        return round(c.range_min + f * (c.range_max - c.range_min))

    # ---- setup ----
    def _check_calibration(self, robot_id: str) -> None:
        bad = []
        for j, c in self.cal.items():
            on_servo = (decode_sm(self.bus.read("Homing_Offset", c.id)),
                        self.bus.read("Min_Position_Limit", c.id), self.bus.read("Max_Position_Limit", c.id))
            if on_servo != (c.homing_offset, c.range_min, c.range_max):
                bad.append(f"{j}: servo {on_servo} vs file {(c.homing_offset, c.range_min, c.range_max)}")
        if bad:
            raise RuntimeError(f"calibration file for '{robot_id}' doesn't match the servos "
                               f"({calibration_path(robot_id)}):\n  " + "\n  ".join(bad))

    def _configure(self) -> None:
        """Same settings as LeRobot's SO101Follower.configure(): fast replies, soft PID, gentle gripper.
        Unlike LeRobot this never drops torque (an arm that is holding a pose would fall): the registers are
        written with EEPROM locked, so they land in RAM, and are re-applied on every connect."""
        not_position = [j for j, i in self.ids.items() if self.bus.read("Operating_Mode", i) != 0]
        if not_position:
            raise RuntimeError(f"{not_position} not in position mode; run `python -m packing.tools.calibrate`")
        for j, i in self.ids.items():
            self.bus.write("Lock", i, 1)
            self.bus.write("Return_Delay_Time", i, 0)
            self.bus.write("Maximum_Acceleration", i, 254)
            self.bus.write("Acceleration", i, 254)
            self.bus.write("P_Coefficient", i, 16)
            self.bus.write("I_Coefficient", i, 0)
            self.bus.write("D_Coefficient", i, 32)
            if j == "gripper":
                self.bus.write("Max_Torque_Limit", i, 500)
                self.bus.write("Protection_Current", i, 250)
                self.bus.write("Overload_Torque", i, 25)
        self.set_torque(True)

    # ---- io ----
    def read(self) -> dict[str, float]:
        raw = self.bus.sync_read("Present_Position", list(self.ids.values()))
        return {j: self.to_units(j, raw[i]) for j, i in self.ids.items()}

    def write(self, pose: dict[str, float]) -> dict[str, float]:
        """Command goal positions; with max_relative_target, each joint moves at most that far from
        where it is now (like LeRobot's ensure_safe_goal_position). Returns the goal actually sent."""
        goal = dict(pose)
        if self.max_relative_target is not None:
            now = self.read()
            cap = float(self.max_relative_target)
            goal = {j: now[j] + max(-cap, min(cap, v - now[j])) for j, v in goal.items()}
        self.bus.sync_write("Goal_Position", {self.ids[j]: self.to_raw(j, v) for j, v in goal.items()})
        return goal

    def set_torque(self, on: bool) -> None:
        if on:  # hold where we are: goal := present before enabling
            present = self.bus.sync_read("Present_Position", list(self.ids.values()))
            self.bus.sync_write("Goal_Position", present)
        for i in self.ids.values():
            self.bus.write("Torque_Enable", i, int(on))
            self.bus.write("Lock", i, int(on))

    def write_register(self, reg: str, joints: list[str], value: int) -> None:
        for j in joints:
            self.bus.write(reg, self.ids[j], value)

    def close(self, release: bool = False) -> None:
        try:
            if release:
                self.set_torque(False)
        finally:
            self.bus.close()
