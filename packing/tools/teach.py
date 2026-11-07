"""Teach pendant: put the arm at a spot and save it as a named pose.

Two ways:
  jog  (default) motors on, move joints with the keyboard
  limp (--limp)  motors off, move the arm by hand, press Enter to save (fast, but the arm sags a few
                 units under its own weight when replayed — re-check with `goto`)
"""

from __future__ import annotations

from ..hardware.arm import Arm
from ..hardware.keyboard import JOG_HELP, JOG_KEYS, raw_terminal, read_key
from ..joints import BODY, clamp_joint
from ..poses import PoseStore

# How far the jog target may run ahead of the actual joint, so the arm stops promptly on key release.
MAX_LEAD = 6.0
STEP_START, STEP_MIN, STEP_MAX = 2.0, 0.5, 10.0
STEP_UP_KEYS = ("+", "=")
STEP_DOWN_KEYS = ("-",)
ENTER_KEYS = ("\n", "\r")
ESCAPE = "\x1b"

HELP = f"  {JOG_HELP}\n  +/- step size   ENTER save pose   ESC skip / cancel"


def _format_pose(pose: dict[str, float]) -> str:
    return " ".join(f"{joint[:8]}={pose[joint]:6.1f}" for joint in BODY)


def _jog(arm: Arm, cmd: dict[str, float], key: str, step: float) -> None:
    """Move the command for the joint bound to `key` by one step, staying within MAX_LEAD of the arm."""
    joint, direction = JOG_KEYS[key]
    actual = arm.read()[joint]
    target = cmd[joint] + direction * step
    near_actual = max(actual - MAX_LEAD, min(actual + MAX_LEAD, target))
    cmd[joint] = clamp_joint(joint, near_actual)


def _teach_one(arm: Arm, poses: PoseStore, name: str, cmd: dict[str, float], step: float,
               limp: bool, fps: int) -> tuple[bool, float]:
    """Interactive loop for one pose. Returns (saved, step) so the step size carries over."""
    print(f"\n=== Teach '{name}' ===" + ("  (move the arm by hand)" if limp else "") + "\n" + HELP)
    if name in poses:
        print(f"  current saved: {_format_pose(poses.get(name))}")
    while True:
        key = read_key(1 / fps)
        if key in ENTER_KEYS:
            actual = arm.read()
            poses.set(name, actual)
            print(f"  saved {name}: {_format_pose(actual)}")
            return True, step
        if key == ESCAPE:
            print(f"  skipped {name}")
            return False, step
        if key in STEP_UP_KEYS:
            step = min(STEP_MAX, step + 1)
        elif key in STEP_DOWN_KEYS:
            step = max(STEP_MIN, step - 1)
        elif key is not None and not limp and key.lower() in JOG_KEYS:
            _jog(arm, cmd, key.lower(), step)
        if limp:
            print("\r  " + _format_pose(arm.read()), end="", flush=True)
        else:
            arm.write(cmd)


def teach(arm: Arm, poses: PoseStore, names: list[str], limp: bool = False, fps: int = 50) -> list[str]:
    """Teach each name in order. Returns the names that were saved."""
    saved: list[str] = []
    if limp:
        input("Motors are about to turn OFF — hold the arm, then press Enter...")
        arm.set_torque(False)
    cmd = arm.read()
    step = STEP_START
    try:
        with raw_terminal():
            for name in names:
                was_saved, step = _teach_one(arm, poses, name, cmd, step, limp, fps)
                if was_saved:
                    saved.append(name)
    finally:
        if limp:
            print("\nHold the arm: motors turning back ON at the current position.")
            arm.write(arm.read())
            arm.set_torque(True)
    return saved
