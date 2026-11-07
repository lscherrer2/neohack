"""Measure motion quality under different servo settings (python -m packing tune).

For each (P gain, acceleration) it runs the same short straight line out and back, records the
commanded and actual joint values every tick, converts both to tip positions with FK, and reports:
  path_err  worst distance of the ACTUAL tip from the ideal straight line (accuracy / joint sync)
  lag       worst distance between where the tip should be now and where it is (tracking)
  jitter    RMS high-frequency wobble of the actual tip (smoothness)
  end_err   tip error after settling (static accuracy, gravity sag)
"""

from __future__ import annotations

import csv
import itertools
import time
from pathlib import Path
from typing import Callable

import numpy as np

from ..hardware.arm import Arm
from ..joints import BODY
from ..motion.controller import Motion
from ..motion.kinematics import ArmModel

Pose = dict[str, float]
Trace = list[tuple[float, Pose, Pose]]  # (time, commanded pose, actual pose) per tick

START: Pose = {"shoulder_pan": 0.0, "shoulder_lift": 10.0, "elbow_flex": 10.0, "wrist_flex": -5.0, "wrist_roll": 0.0}
OFFSET_M = (0.03, -0.03, 0.0)  # 42 mm diagonal: needs every body joint except roll
MAX_GOAL_IK_ERR_M = 0.001
SERVO_SETTLE_S = 0.3  # wait after changing servo settings
TRACE_SETTLE_S = 0.5  # wait after the test line before measuring end error
SMOOTHING_WINDOW = 7  # ticks, for the jitter measurement
START_SPEED = 30  # units/s for moves to the start pose


def _dist_to_segment(p: np.ndarray, a: np.ndarray, b: np.ndarray) -> float:
    """Distance from point `p` to the segment a-b."""
    ab = b - a
    t = np.clip(np.dot(p - a, ab) / np.dot(ab, ab), 0, 1)
    return float(np.linalg.norm(p - (a + t * ab)))


def _metrics(model: ArmModel, trace: Trace, a: np.ndarray, b: np.ndarray,
             settled_pose: Pose, target: Pose) -> dict:
    """Quality numbers (mm) for one recorded out-and-back run along the segment a-b."""
    t = np.array([row[0] for row in trace])
    tip_act = np.array([model.tip(row[2]) for row in trace])
    tip_cmd = np.array([model.tip(row[1]) for row in trace])
    half = SMOOTHING_WINDOW // 2
    smooth = np.array([tip_act[max(0, i - half): i + half + 1].mean(0) for i in range(len(tip_act))])
    lag_units = {j: max(abs(row[1][j] - row[2][j]) for row in trace) for j in BODY}
    return {
        "path_err": max(_dist_to_segment(p, a, b) for p in tip_act) * 1000,
        "lag": float(np.linalg.norm(tip_act - tip_cmd, axis=1).max()) * 1000,
        "jitter": float(np.sqrt((np.linalg.norm(tip_act - smooth, axis=1) ** 2).mean())) * 1000,
        "end_err": float(np.linalg.norm(model.tip(settled_pose) - model.tip(target))) * 1000,
        "hz": (len(t) - 1) / (t[-1] - t[0]) if len(t) > 1 else 0.0,
        "worst_joint": max(lag_units, key=lag_units.get),
        "worst_joint_lag": max(lag_units.values()),
    }


def _write_csv(path: Path, trace: Trace) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["t", *[f"cmd_{j}" for j in BODY], *[f"act_{j}" for j in BODY]])
        for t, cmd, actual in trace:
            writer.writerow([f"{t:.4f}", *[f"{cmd[j]:.2f}" for j in BODY], *[f"{actual[j]:.2f}" for j in BODY]])


def _result_line(m: dict) -> str:
    return (f"  P={m['p']:<3} accel={m['accel']:<4} sag={m['sag']:<4} path_err={m['path_err']:5.1f}mm  "
            f"lag={m['lag']:5.1f}mm  jitter={m['jitter']:4.2f}mm  end_err={m['end_err']:4.1f}mm  "
            f"{m['hz']:.0f}Hz  (worst joint {m['worst_joint']} {m['worst_joint_lag']:.1f})")


def run_tune(arm: Arm, motion: Motion, model: ArmModel, p_gains: list[int], accels: list[int],
             speed_mm_s: float = 50.0, reps: int = 2, out_dir: Path | None = None,
             log: Callable[[str], None] = print, sag_gains: tuple[float, ...] | list[float] = (0.0,),
             restore: tuple[int, int] = (16, 254)) -> list[dict]:
    """Run the test line for every (P, accel, sag gain) combination and report the metrics.
    Settings in `restore` (P, accel) are applied again at the end. Writes one CSV per setting
    into `out_dir` if given."""
    goal, err = model.ik(START, model.offset(START, OFFSET_M))
    if err > MAX_GOAL_IK_ERR_M:
        raise RuntimeError("test line unreachable")
    out_path = model.line(START, goal)
    back_path = list(reversed(out_path))
    a, b = model.tip(START), model.tip(goal)
    length = float(np.linalg.norm(b - a))
    duration = length * 1000 / speed_mm_s
    log(f"test: {length * 1000:.0f} mm straight line, {duration:.2f} s each way, x{reps} out+back per setting")
    motion.move_to(START, speed=START_SPEED)
    results = []
    try:
        for p, accel, sag in itertools.product(p_gains, accels, sag_gains):
            motion.cfg.sag_gain = sag
            arm.tune(p_gain=p, accel=accel)
            time.sleep(SERVO_SETTLE_S)
            motion.move_to(START, speed=START_SPEED)
            motion.trace = []
            for _ in range(reps):
                motion.follow(out_path, duration, "tune-out")
                motion.follow(back_path, duration, "tune-back")
            trace, motion.trace = motion.trace, None
            time.sleep(TRACE_SETTLE_S)
            metrics = _metrics(model, trace, a, b, motion.read(), START)
            metrics.update(p=p, accel=accel, sag=sag)
            results.append(metrics)
            log(_result_line(metrics))
            if out_dir:
                _write_csv(out_dir / f"tune_P{p}_A{accel}_S{sag}.csv", trace)
    finally:
        motion.trace = None
        arm.tune(p_gain=restore[0], accel=restore[1])  # back to configured settings
    best = min(results, key=lambda r: r["path_err"] + 2 * r["jitter"])
    log(f"best: P={best['p']} accel={best['accel']} sag={best['sag']}  "
        "-> motion.servo_p / servo_accel / sag_gain in cell.yaml")
    return results
