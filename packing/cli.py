"""python -m packing <command>   (run from the repo root with .venv: `source .venv/bin/activate`)

Setup:     check | teach NAME... | teach-all [--missing] | poses | goto NAME | home | release
Tray:      tray | tray-calibrate
Inventory: restock | status | events | set-part ID STATUS | set-box ID LOCATION [--status S] | clear-output
Run:       demo [--loops N] | pack [--count N] | move-box BOX_ID LOCATION
Add --sim to any command to run against a simulated arm (no hardware).

Commands are grouped by what they need:
  inventory commands (check, poses, restock, ...) only touch files and the database;
  arm-direct commands (teach, release) use the bare arm;
  motion commands (home, goto, ...) also get a `Motion` and a `Cell` with the servo settings applied.
"""

from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Callable

from .cell.inventory import SHIPPED, Inventory, InventoryError
from .cell.procedures import Cell, CellError
from .config import CellConfig, load_config
from .hardware.arm import Arm, FakeArm, RealArm
from .joints import BODY
from .motion.controller import Motion, MotionError
from .motion.kinematics import ArmModel, PathError, gravity_feedforward, load_ranges
from .poses import PoseNotTaughtError, PoseStore

PACKING_DIR = Path(__file__).parent
DEFAULT_CONFIG = PACKING_DIR / "cell.yaml"
DEFAULT_DEMO = PACKING_DIR / "demo_waypoints.yaml"

EXIT_OK, EXIT_STOPPED, EXIT_INTERRUPTED = 0, 2, 130
LINE_SPEED_M_S = 0.04  # average tip speed of `goto --line`
LINE_MIN_DURATION_S = 0.5
SIM_GRIPPER_BLOCK = 15  # simulated object: gripper stops this far from `closed`


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="python -m packing", description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--config", default=str(DEFAULT_CONFIG))
    p.add_argument("--sim", action="store_true", help="simulated arm, no hardware")
    sub = p.add_subparsers(dest="cmd", required=True)
    sub.add_parser("check", help="list poses that still need teaching")
    t = sub.add_parser("teach", help="teach named poses, e.g. part_A1.above part_A1.at")
    t.add_argument("names", nargs="+")
    t.add_argument("--limp", action="store_true", help="motors off, move by hand")
    ta = sub.add_parser("teach-all", help="teach every pose the config needs, in order")
    ta.add_argument("--missing", action="store_true", help="only poses not taught yet")
    ta.add_argument("--limp", action="store_true")
    sub.add_parser("poses", help="list taught poses")
    g = sub.add_parser("goto", help="move to a pose (or location: goes to L.above)")
    g.add_argument("name")
    g.add_argument("--line", action="store_true", help="straight-line tip path (IK) instead of joint move")
    sub.add_parser("where", help="print joint values and gripper-tip position")
    sub.add_parser("home", help="move to the travel/home pose")
    sub.add_parser("release", help="turn motors off (hold the arm!)")
    sub.add_parser("restock", help="after refilling: mark empty part slots / box supply spots as full")
    sub.add_parser("tray", help="parts tray: pockets from the DXF, calibration status, which pockets are full")
    sub.add_parser("tray-calibrate", help="motors off: touch the tray reference pockets with the tip, Enter each")
    sub.add_parser("status", help="inventory summary")
    e = sub.add_parser("events", help="recent inventory events")
    e.add_argument("-n", type=int, default=20)
    sp = sub.add_parser("set-part", help="manual fix: set a part's status")
    sp.add_argument("id", type=int)
    sp.add_argument("status", choices=["available", "packed", "missing"])
    sp.add_argument("--location")
    sb = sub.add_parser("set-box", help="manual fix: say where a box is")
    sb.add_argument("id")
    sb.add_argument("location")
    sb.add_argument("--status", choices=["empty", "packing", "packed"])
    sub.add_parser("clear-output", help="boxes on output spots were taken away")
    dm = sub.add_parser("demo", help="slow waypoint demo (packing/demo_waypoints.yaml)")
    dm.add_argument("--file", default=str(DEFAULT_DEMO))
    dm.add_argument("--loops", type=int, default=1)
    dm.add_argument("--speed", type=float, default=1.0, help="speed multiplier, e.g. 0.5 or 1.5")
    tu = sub.add_parser("tune", help="measure motion quality under different servo settings")
    tu.add_argument("--p", type=int, nargs="+", default=None,
                    help="servo P gains to try (default: motion.servo_p from cell.yaml)")
    tu.add_argument("--accel", type=int, nargs="+", default=[20], help="servo accelerations to try")
    tu.add_argument("--speed", type=float, default=50.0, help="tip speed mm/s")
    tu.add_argument("--sag", type=float, nargs="+", default=[0.0, 0.08], help="sag-compensation gains to try")
    pk = sub.add_parser("pack", help="package boxes automatically (resumes if interrupted)")
    pk.add_argument("--count", type=int, default=1)
    mb = sub.add_parser("move-box", help="move one box to a location")
    mb.add_argument("id")
    mb.add_argument("location")
    return p


@dataclass
class Session:
    """Everything a command handler needs. `release_on_exit` makes the arm go limp when it closes."""

    args: argparse.Namespace
    cfg: CellConfig
    poses: PoseStore
    inventory: Inventory
    release_on_exit: bool = False


@dataclass
class Rig:
    """The arm plus the motion and cell layers built on it."""

    arm: Arm
    motion: Motion
    cell: Cell


# ---------- commands that don't need the arm ----------
def cmd_check(s: Session) -> int:
    required = s.cfg.required_poses()
    missing = s.poses.missing(required)
    print(f"{len(required) - len(missing)}/{len(required)} poses taught.")
    if missing:
        print("Still to teach:\n  " + "\n  ".join(missing))
        print("Run: python -m packing teach-all --missing")
    return 0 if not missing else 1


def cmd_poses(s: Session) -> int:
    for name in s.poses.names():
        print(f"{name:<28} " + " ".join(f"{v:7.1f}" for v in s.poses.get(name).values()))
    return EXIT_OK


def cmd_restock(s: Session) -> int:
    parts, boxes = s.inventory.restock(s.cfg.stock_slots, s.cfg.box_supply)
    print(f"added {parts} parts, {boxes} boxes")
    print(s.inventory.summary())
    return EXIT_OK


def cmd_status(s: Session) -> int:
    print(s.inventory.summary())
    return EXIT_OK


def cmd_events(s: Session) -> int:
    for row in s.inventory.events(s.args.n):
        print(f"{row['ts']}  {row['kind']:<13} part={row['part_id']} box={row['box_id']}  {row['detail']}")
    return EXIT_OK


def cmd_set_part(s: Session) -> int:
    s.inventory.set_part(s.args.id, s.args.status, s.args.location)
    print(s.inventory.summary())
    return EXIT_OK


def cmd_set_box(s: Session) -> int:
    s.inventory.move_box(s.args.id, s.args.location, s.args.status)
    print(s.inventory.summary())
    return EXIT_OK


def cmd_clear_output(s: Session) -> int:
    for location in s.cfg.box_output:
        box = s.inventory.box_at(location)
        if box:
            s.inventory.move_box(box.id, SHIPPED)
    print(s.inventory.summary())
    return EXIT_OK


# ---------- commands that use the bare arm ----------
def cmd_teach(s: Session, arm: Arm) -> int:
    from .tools.teach import teach

    if s.args.cmd == "teach":
        names = s.args.names
    else:
        names = s.cfg.required_poses()
        if s.args.missing:
            names = s.poses.missing(names)
    teach(arm, s.poses, names, limp=s.args.limp)
    return EXIT_OK


def cmd_release(s: Session, arm: Arm) -> int:
    input("Hold the arm, then press Enter to turn the motors off...")
    s.release_on_exit = True
    return EXIT_OK


def _tray_or_exit(s: Session):
    if not s.cfg.tray:
        sys.exit("No `tray:` section in the config.")
    return s.cfg.tray


def _pan_xy(cfg: CellConfig):
    try:
        return _load_arm_model(cfg).pan_xy
    except CellError:
        return None


def cmd_tray(s: Session) -> int:
    from .tools.tray_calibrate import summarize

    tray = _tray_or_exit(s)
    layout = tray.layout()
    full = s.inventory.full_slots()
    print(f"{tray.dxf.name}: {len(layout.pockets)} pockets")
    for t in tray.types:
        ids = [p.id for p in layout.pockets if p.sku == t.sku]
        marks = "".join("#" if pid in full else "." for pid in ids)
        print(f"  {t.sku:<11} {t.name:<15} {len(ids):>2}  [{marks}]  ({sum(pid in full for pid in ids)} full)")
    for u in layout.unmatched:
        print(f"  unmatched DXF feature: {u}")
    cal = tray.calibration()
    if not cal:
        print("not calibrated: python -m packing tray-calibrate")
        return EXIT_OK
    print(f"calibrated {cal.created}")
    for line in summarize(cal, layout, _pan_xy(s.cfg)):
        print("  " + line)
    return EXIT_OK


def cmd_tray_calibrate(s: Session, arm: Arm) -> int:
    from .tools.tray_calibrate import run_tray_calibration

    tray = _tray_or_exit(s)
    model = _load_arm_model(s.cfg)
    cal = run_tray_calibration(arm, model.tip, tray, s.cfg.catalog, pan_xy=model.pan_xy)
    return EXIT_OK if cal else EXIT_STOPPED


# ---------- commands that move the arm ----------
def cmd_home(s: Session, rig: Rig) -> int:
    if s.cfg.travel_pose:
        rig.cell.travel()
    else:
        rig.motion.move_to("home")
    return EXIT_OK


def cmd_goto(s: Session, rig: Rig) -> int:
    name = s.args.name
    if "." not in name and name not in s.poses:
        name = f"{name}.above"  # a bare location name means its safe approach pose
    if not s.args.line:
        rig.motion.move_to(name)
        return EXIT_OK
    model = _load_arm_model(s.cfg)
    start = {joint: rig.motion.cmd[joint] for joint in BODY}
    path = model.line(start, s.poses.get(name))
    length = model.path_length([model.tip(pose) for pose in path])
    rig.motion.follow(path, max(LINE_MIN_DURATION_S, length / LINE_SPEED_M_S), name)
    return EXIT_OK


def cmd_where(s: Session, rig: Rig) -> int:
    pose = rig.motion.read()
    print("joints:", {joint: round(value, 1) for joint, value in pose.items()})
    print("tip (mm, x fwd / y left / z up):", (_load_arm_model(s.cfg).tip(pose) * 1000).round(1))
    return EXIT_OK


def cmd_tune(s: Session, rig: Rig) -> int:
    from .tools.tune import run_tune

    motion_cfg = s.cfg.motion
    p_gains = s.args.p or [motion_cfg.servo_p or 16]
    run_tune(rig.arm, rig.motion, _load_arm_model(s.cfg), p_gains, s.args.accel, s.args.speed,
             out_dir=None if s.args.sim else s.cfg.poses_file.parent / "tune",
             sag_gains=s.args.sag, restore=(motion_cfg.servo_p or 16, motion_cfg.servo_accel or 254))
    return EXIT_OK


def cmd_demo(s: Session, rig: Rig) -> int:
    from .tools.demo import load_demo, run_demo

    model = _load_arm_model(s.cfg)
    demo = load_demo(Path(s.args.file), s.poses, model)
    run_demo(rig.motion, demo, model, s.args.loops, s.args.speed)
    print("Demo complete — arm holding at the last waypoint.")
    return EXIT_OK


def cmd_move_box(s: Session, rig: Rig) -> int:
    rig.cell.check_ready()
    rig.cell.transfer_box(s.args.id, s.args.location)
    rig.cell.travel()
    return EXIT_OK


def cmd_pack(s: Session, rig: Rig) -> int:
    missing = s.poses.missing(s.cfg.required_poses())
    if missing:
        raise CellError(f"{len(missing)} poses not taught yet (run `check`)")
    for i in range(s.args.count):
        box_id = rig.cell.package_box()
        print(f"[{i + 1}/{s.args.count}] packed {box_id}")
    print(s.inventory.summary())
    return EXIT_OK


INVENTORY_COMMANDS: dict[str, Callable[[Session], int]] = {
    "check": cmd_check,
    "poses": cmd_poses,
    "restock": cmd_restock,
    "status": cmd_status,
    "events": cmd_events,
    "set-part": cmd_set_part,
    "set-box": cmd_set_box,
    "clear-output": cmd_clear_output,
    "tray": cmd_tray,
}
ARM_COMMANDS: dict[str, Callable[[Session, Arm], int]] = {
    "teach": cmd_teach,
    "teach-all": cmd_teach,
    "release": cmd_release,
    "tray-calibrate": cmd_tray_calibrate,
}
MOTION_COMMANDS: dict[str, Callable[[Session, Rig], int]] = {
    "home": cmd_home,
    "goto": cmd_goto,
    "where": cmd_where,
    "tune": cmd_tune,
    "demo": cmd_demo,
    "move-box": cmd_move_box,
    "pack": cmd_pack,
}


# ---------- setup helpers ----------
def _open_session(args: argparse.Namespace) -> Session:
    config_path = Path(args.config)
    if not config_path.exists():
        sys.exit(f"Config {config_path} not found (copy packing/cell.example.yaml to packing/cell.yaml).")
    cfg = load_config(config_path)
    if args.sim:
        # Simulated runs must never touch the real inventory or taught poses.
        if str(cfg.inventory_db) != ":memory:":
            cfg.inventory_db = cfg.inventory_db.with_name("sim_" + cfg.inventory_db.name)
        cfg.poses_file = cfg.poses_file.with_name("sim_" + cfg.poses_file.name)
        if cfg.tray:
            cfg.tray.calibration_file = cfg.tray.calibration_file.with_name("sim_" + cfg.tray.calibration_file.name)
    return Session(args, cfg, PoseStore(cfg.poses_file), Inventory(cfg.inventory_db))


def _open_arm(cfg: CellConfig, sim: bool) -> Arm:
    if not sim:
        return RealArm(cfg.port, cfg.robot_id, cfg.motion.max_step)
    arm = FakeArm(closed=cfg.gripper.closed, opened=cfg.gripper.open)
    closing_up = cfg.gripper.closed < cfg.gripper.open
    arm.gripper_block = cfg.gripper.closed + (SIM_GRIPPER_BLOCK if closing_up else -SIM_GRIPPER_BLOCK)
    return arm


def _no_sleep(seconds: float) -> None:
    """Sleep replacement for --sim: simulated moves run as fast as possible."""


def _build_rig(s: Session, arm: Arm) -> Rig:
    arm.tune(p_gain=s.cfg.motion.servo_p, accel=s.cfg.motion.servo_accel)
    gravity = None if s.args.sim else gravity_feedforward(s.cfg.robot_id, s.cfg.motion.gravity_gain)
    motion = Motion(arm, s.poses, s.cfg.motion, s.cfg.gripper, sleep=_no_sleep if s.args.sim else time.sleep,
                    gravity=gravity)
    return Rig(arm, motion, Cell(s.cfg, motion, s.inventory))


def _load_arm_model(cfg: CellConfig) -> ArmModel:
    try:
        return ArmModel(load_ranges(cfg.robot_id))
    except ImportError as e:
        raise CellError("straight-line moves need placo: pip install -r requirements.txt") from e


def _safe_hold(arm: Arm) -> None:
    """Best effort: command the arm to stay where it is."""
    try:
        arm.write(arm.read())
    except Exception:
        pass


def _run_with_arm(s: Session) -> int:
    """Run an arm command; on Ctrl+C or a safety stop, hold position and print the inventory."""
    arm = _open_arm(s.cfg, s.args.sim)
    try:
        if s.args.cmd in ARM_COMMANDS:
            return ARM_COMMANDS[s.args.cmd](s, arm)
        return MOTION_COMMANDS[s.args.cmd](s, _build_rig(s, arm))
    except KeyboardInterrupt:
        print("\nStopped by Ctrl+C — arm holding position.")
        _safe_hold(arm)
        print(s.inventory.summary())
        return EXIT_INTERRUPTED
    except (MotionError, CellError, InventoryError, PathError, PoseNotTaughtError, ValueError) as e:
        print(f"\nSTOPPED: {e}")
        _safe_hold(arm)
        print(s.inventory.summary())
        return EXIT_STOPPED
    finally:
        arm.close(release=s.release_on_exit)


def main(argv: list[str] | None = None) -> int:
    session = _open_session(build_parser().parse_args(argv))
    handler = INVENTORY_COMMANDS.get(session.args.cmd)
    if handler:
        return handler(session)
    return _run_with_arm(session)
