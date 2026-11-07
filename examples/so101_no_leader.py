"""Drive an SO-101 follower arm WITHOUT a leader arm.

Usage (after calibrating with lerobot-calibrate):
    python examples/so101_no_leader.py --port /dev/cu.usbmodemXXXX demo   # gentle wiggle of each joint
    python examples/so101_no_leader.py --port /dev/cu.usbmodemXXXX keys   # drive joints from the keyboard
    python examples/so101_no_leader.py --port /dev/cu.usbmodemXXXX --keep-torque home   # glide to the home pose and hold

Units: body joints are -100..100 (calibrated range), gripper is 0..100.
Ctrl+C at any time stops; torque is disabled on exit (the arm will go limp — support it!).

This script is deliberately standalone (it does not use the `packing/` package).
"""

import argparse
import select
import sys
import termios
import time
import tty

from lerobot.robots.so101_follower import SO101Follower, SO101FollowerConfig

BODY = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
JOINTS = [*BODY, "gripper"]
KEYS = {  # key: (joint, direction)
    "q": ("shoulder_pan", +1), "a": ("shoulder_pan", -1),
    "w": ("shoulder_lift", +1), "s": ("shoulder_lift", -1),
    "e": ("elbow_flex", +1), "d": ("elbow_flex", -1),
    "r": ("wrist_flex", +1), "f": ("wrist_flex", -1),
    "t": ("wrist_roll", +1), "g": ("wrist_roll", -1),
    "y": ("gripper", +1), "h": ("gripper", -1),
}
FPS = 50
LEAD = 15.0  # default max distance the keyboard target may lead the actual joint (must be < MAX_STEP)
MAX_STEP = 20.0  # library safety cap per command (max_relative_target)
STEP_MIN, STEP_MAX = 0.5, 10.0
HOME = dict.fromkeys(JOINTS, 0.0)  # calibration mid-pose, gripper closed

Pose = dict[str, float]


def clamp(joint: str, value: float) -> float:
    low = 0.0 if joint == "gripper" else -100.0
    return max(low, min(100.0, value))


def round_pose(pose: Pose) -> Pose:
    return {joint: round(value, 1) for joint, value in pose.items()}


def current_pose(robot) -> Pose:
    obs = robot.get_observation()
    return {joint: float(obs[f"{joint}.pos"]) for joint in JOINTS}


def send(robot, pose: Pose) -> None:
    robot.send_action({f"{joint}.pos": clamp(joint, value) for joint, value in pose.items()})


def move_to(robot, start: Pose, target: Pose, seconds: float) -> Pose:
    """Linear joint-space glide from `start` to `target` over `seconds`."""
    steps = max(1, int(seconds * FPS))
    for i in range(1, steps + 1):
        a = i / steps
        send(robot, {j: start[j] + (target[j] - start[j]) * a for j in JOINTS})
        time.sleep(1 / FPS)
    return dict(target)


def demo(robot, amp: float) -> None:
    """Wiggle each joint by +amp, -amp, then back to where it started."""
    start = current_pose(robot)
    pose = dict(start)
    print("Start pose:", round_pose(start))
    for joint in JOINTS:
        print(f"Moving {joint} ...")
        for delta in (+amp, -amp, 0):
            target = dict(start)
            target[joint] = clamp(joint, start[joint] + delta)
            pose = move_to(robot, pose, target, 1.0)
    print("Done, back at start pose.")


def home(robot, speed: float = 25.0) -> None:
    """Glide to HOME at ~`speed` units/s (based on the joint that has farthest to go)."""
    start = current_pose(robot)
    distance = max(abs(HOME[joint] - start[joint]) for joint in JOINTS)
    print("From:", round_pose(start))
    move_to(robot, start, HOME, max(1.0, distance / speed))
    time.sleep(0.5)
    print("Home:", round_pose(current_pose(robot)))


def keys(robot, step: float, lead: float = LEAD) -> None:
    """Drive the joints from the keyboard until x is pressed."""
    pose = current_pose(robot)
    print(__doc__.split("Units")[0])
    print("Keys (up/down):  q/a pan   w/s shoulder   e/d elbow   r/f wrist_flex   t/g wrist_roll   y/h gripper")
    print("  +/- change step size, p print pose, x quit\n")
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        while True:
            if select.select([sys.stdin], [], [], 1 / FPS)[0]:
                k = sys.stdin.read(1).lower()
                if k == "x":
                    break
                if k in KEYS:
                    joint, direction = KEYS[k]
                    # Don't let the target run ahead of the real joint (it lags under load / key-repeat),
                    # so the arm stops promptly when the key is released and the safety cap never fires.
                    actual = float(robot.bus.read("Present_Position", joint))
                    target = pose[joint] + direction * step
                    pose[joint] = clamp(joint, max(actual - lead, min(actual + lead, target)))
                elif k in "+=":
                    step = min(STEP_MAX, step + 1)
                    print(f"step = {step}")
                elif k == "-":
                    step = max(STEP_MIN, step - 1)
                    print(f"step = {step}")
                elif k == "p":
                    print(round_pose(pose))
            send(robot, pose)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def run_mode(robot, args: argparse.Namespace) -> None:
    if args.mode == "demo":
        demo(robot, args.amp)
    elif args.mode == "keys":
        keys(robot, args.step, args.lead)
    else:
        home(robot)


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser()
    p.add_argument("mode", choices=["demo", "keys", "home"])
    p.add_argument("--port", required=True)
    p.add_argument("--id", default="my_awesome_follower_arm")
    p.add_argument("--amp", type=float, default=15.0, help="demo wiggle size")
    p.add_argument("--step", type=float, default=2.0, help="keyboard step size")
    p.add_argument("--p-gain", type=int, default=32,
                   help="servo stiffness for body joints (LeRobot default 16, factory 32). "
                        "Higher = stronger, more jitter/heat. RAM only, resets on power-off/reconnect")
    p.add_argument("--lead", type=float, default=LEAD, help="keys mode: how far the target may lead the joint")
    p.add_argument("--keep-torque", action="store_true", help="leave motors holding the pose on exit")
    return p.parse_args()


def main() -> None:
    args = parse_args()

    # max_relative_target caps how far a single command can jump (safety).
    robot = SO101Follower(SO101FollowerConfig(port=args.port, id=args.id, max_relative_target=MAX_STEP))
    if not robot.calibration:
        sys.exit(f"No calibration file for id '{args.id}'. Run lerobot-calibrate first.")
    robot.connect(calibrate=False)
    if not 0 < args.lead < MAX_STEP:
        sys.exit(f"--lead must be between 0 and {MAX_STEP}")
    # Torque is on, so the EEPROM is locked: this lands in RAM only (until power-off / next connect).
    for joint in BODY:
        robot.bus.write("P_Coefficient", joint, args.p_gain)
    print(f"Stiffness (P) = {args.p_gain} on body joints")
    try:
        run_mode(robot, args)
    except KeyboardInterrupt:
        pass
    finally:
        if args.keep_torque:
            robot.bus.disconnect(disable_torque=False)
            print("Exited; motors still holding the pose.")
        else:
            try:
                input("\nHold the arm (torque turns off) and press Enter to release...")
            except EOFError:
                pass
            robot.disconnect()


if __name__ == "__main__":
    main()
