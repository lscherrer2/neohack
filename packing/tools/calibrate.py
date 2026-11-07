"""SO-101 calibration, no LeRobot (same procedure and EEPROM values as `lerobot-calibrate`).

Torque stays OFF the whole time (the arm is limp - support it); you move the arm by hand.

    python -m packing.tools.calibrate scan                  # read-only health check of servos 1-6
    python -m packing.tools.calibrate backup                # save what's on the servos now
    python -m packing.tools.calibrate center                # arm held in the middle pose -> homing offsets
    python -m packing.tools.calibrate range [SECONDS] [JOINT ...]   # move joints end to end -> limits + file
    python -m packing.tools.calibrate restore FILE          # write a saved calibration back to the servos

`range` with joint names re-records just those joints (keeps the others from the last attempt).
Recalibrating changes what every taught pose means - re-teach afterwards.
"""

from __future__ import annotations

import argparse
import json
import sys
import time

from ..config import load_config
from ..hardware.so101 import (CALIBRATION_DIR, MOTOR_IDS, FeetechBus, JointCalibration, decode_sm, encode_sm,
                              save_calibration)
from ..cli import DEFAULT_CONFIG

HALF_TURN = 2047
MIN_SPAN = 500  # a joint that moved less than this during `range` wasn't really moved
CENTER_FILE = CALIBRATION_DIR / ".center_pending.json"
PARTIAL_FILE = CALIBRATION_DIR / ".range_partial.json"


def read_calibration(bus: FeetechBus) -> dict[str, JointCalibration]:
    return {j: JointCalibration(id=i, homing_offset=decode_sm(bus.read("Homing_Offset", i)),
                                range_min=bus.read("Min_Position_Limit", i), range_max=bus.read("Max_Position_Limit", i))
            for j, i in MOTOR_IDS.items()}


def unlock(bus: FeetechBus) -> None:
    """Torque off + EEPROM unlocked + position mode."""
    for i in MOTOR_IDS.values():
        bus.write("Torque_Enable", i, 0)
        bus.write("Lock", i, 0)
        bus.write("Operating_Mode", i, 0)


def lock(bus: FeetechBus) -> None:
    for i in MOTOR_IDS.values():
        bus.write("Lock", i, 1)


def write_calibration(bus: FeetechBus, cal: dict[str, JointCalibration]) -> None:
    unlock(bus)
    for c in cal.values():
        bus.write("Homing_Offset", c.id, encode_sm(c.homing_offset))
        bus.write("Min_Position_Limit", c.id, c.range_min)
        bus.write("Max_Position_Limit", c.id, c.range_max)
    lock(bus)


def cmd_scan(bus: FeetechBus, robot_id: str) -> None:
    print(f"{'id':>2} {'joint':<14} {'model':>5} {'pos':>5} {'torque':>6} {'V':>5} {'C':>3} {'offset':>6} {'min':>5} {'max':>5}")
    for j, i in MOTOR_IDS.items():
        model = bus.ping(i)
        if model is None:
            print(f"{i:>2} {j:<14} -- no response")
            continue
        print(f"{i:>2} {j:<14} {model:>5} {bus.read('Present_Position', i):>5} {bus.read('Torque_Enable', i):>6} "
              f"{bus.read('Present_Voltage', i) / 10:>5.1f} {bus.read('Present_Temperature', i):>3} "
              f"{decode_sm(bus.read('Homing_Offset', i)):>6} {bus.read('Min_Position_Limit', i):>5} "
              f"{bus.read('Max_Position_Limit', i):>5}")


def cmd_backup(bus: FeetechBus, robot_id: str) -> None:
    cal = read_calibration(bus)
    out = CALIBRATION_DIR / f"{robot_id}.backup_{time.strftime('%Y%m%d_%H%M%S')}.json"
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps({j: vars(c) for j, c in cal.items()}, indent=4) + "\n")
    print(f"saved {out}")


def cmd_center(bus: FeetechBus, robot_id: str) -> None:
    unlock(bus)
    for i in MOTOR_IDS.values():  # clear old calibration so we read raw encoder values
        bus.write("Homing_Offset", i, 0)
        bus.write("Min_Position_Limit", i, 0)
        bus.write("Max_Position_Limit", i, 4095)
    time.sleep(0.3)  # Present_Position is stale right after a Homing_Offset write
    offsets = {}
    for j, i in MOTOR_IDS.items():
        offsets[j] = bus.read("Present_Position", i) - HALF_TURN
        bus.write("Homing_Offset", i, encode_sm(offsets[j]))
    time.sleep(0.3)
    lock(bus)
    print(f"{'joint':<14} {'offset':>6} {'reads now':>9}")
    for j, i in MOTOR_IDS.items():
        print(f"{j:<14} {offsets[j]:>6} {bus.read('Present_Position', i):>9}   (target {HALF_TURN})")
    CENTER_FILE.parent.mkdir(parents=True, exist_ok=True)
    CENTER_FILE.write_text(json.dumps(offsets))
    print("center done -> next: range")


def cmd_range(bus: FeetechBus, robot_id: str, seconds: float, only: list[str]) -> int:
    if not CENTER_FILE.exists():
        sys.exit("run `center` first")
    offsets = json.loads(CENTER_FILE.read_text())
    lo, hi = dict.fromkeys(MOTOR_IDS, 4095), dict.fromkeys(MOTOR_IDS, 0)
    record = list(MOTOR_IDS)
    if only:
        bad = [j for j in only if j not in MOTOR_IDS]
        if bad:
            sys.exit(f"unknown joints {bad}; joints: {', '.join(MOTOR_IDS)}")
        if not PARTIAL_FILE.exists():
            sys.exit("no earlier attempt saved; run a full `range` first")
        prev = json.loads(PARTIAL_FILE.read_text())
        lo, hi, record = prev["lo"], prev["hi"], only
        for j in only:
            lo[j], hi[j] = 4095, 0

    print(f"recording {', '.join(record)} for {seconds:.0f}s - move slowly end to end...", flush=True)
    ids = [MOTOR_IDS[j] for j in record]
    t_end, next_print = time.time() + seconds, 0.0
    while time.time() < t_end:
        try:
            raw = bus.sync_read("Present_Position", ids)
        except IOError:
            continue
        for j in record:
            p = raw[MOTOR_IDS[j]]
            lo[j], hi[j] = min(lo[j], p), max(hi[j], p)
        if time.time() >= next_print:
            next_print = time.time() + 5
            print(f"[{t_end - time.time():4.0f}s left] " + "  ".join(f"{j}:{max(0, hi[j] - lo[j])}" for j in record),
                  flush=True)
        time.sleep(0.01)

    print(f"\n{'joint':<14} {'min':>5} {'max':>5} {'span':>5}")
    for j in MOTOR_IDS:
        print(f"{j:<14} {lo[j]:>5} {hi[j]:>5} {hi[j] - lo[j]:>5}")
    small = [j for j in MOTOR_IDS if hi[j] - lo[j] < MIN_SPAN]
    if small:
        PARTIAL_FILE.write_text(json.dumps({"lo": lo, "hi": hi}))
        print(f"\nbarely moved: {', '.join(small)} - nothing written. Re-record just those:\n"
              f"  python -m packing.tools.calibrate range 20 {' '.join(small)}")
        return 1

    cal = {j: JointCalibration(id=i, homing_offset=offsets[j], range_min=lo[j], range_max=hi[j])
           for j, i in MOTOR_IDS.items()}
    write_calibration(bus, cal)
    path = save_calibration(robot_id, cal)
    CENTER_FILE.unlink()
    PARTIAL_FILE.unlink(missing_ok=True)
    print(f"\nwritten to the servos and {path}")
    return 0


def cmd_restore(bus: FeetechBus, robot_id: str, file: str) -> None:
    data = json.loads(open(file).read())
    cal = {j: JointCalibration(**{k: data[j][k] for k in ("id", "homing_offset", "range_min", "range_max")})
           for j in MOTOR_IDS}
    write_calibration(bus, cal)
    path = save_calibration(robot_id, cal)
    print(f"restored {file} to the servos and {path}")


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(prog="python -m packing.tools.calibrate", description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default=str(DEFAULT_CONFIG))
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("scan")
    sub.add_parser("backup")
    sub.add_parser("center")
    r = sub.add_parser("range")
    r.add_argument("seconds", nargs="?", type=float, default=60)
    r.add_argument("joints", nargs="*")
    rs = sub.add_parser("restore")
    rs.add_argument("file")
    a = ap.parse_args(argv)

    cfg = load_config(a.config)
    bus = FeetechBus(cfg.port)
    try:
        if a.cmd == "range":
            return cmd_range(bus, cfg.robot_id, a.seconds, a.joints)
        if a.cmd == "restore":
            cmd_restore(bus, cfg.robot_id, a.file)
        else:
            {"scan": cmd_scan, "backup": cmd_backup, "center": cmd_center}[a.cmd](bus, cfg.robot_id)
        return 0
    finally:
        bus.close()


if __name__ == "__main__":
    sys.exit(main())
