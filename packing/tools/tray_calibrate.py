"""Tray calibration by hand: motors off, guide the gripper tip to each reference pocket, press Enter.

The tip point is the kinematic tool point (`gripper_frame_link`, between the fingertips with the gripper
closed). Touch each pocket's center at the tray face. Hold the gripper roughly the way it will be when
picking (pointing into the tray) so any small tool-point error is the same during calibration and picking.
"""

from __future__ import annotations

import time
from typing import Callable

import numpy as np

from ..config import TrayCfg
from ..hardware.arm import Arm
from ..tray.calibration import CalibrationError, TrayCalibration, approach_misalignment_deg, calibrate, checks, tilt_deg
from ..tray.layout import TrayLayout

SAMPLES = 5  # joint reads averaged per point (hand tremor)


def describe_pocket(layout: TrayLayout, pocket_id: str, names: dict[str, str]) -> str:
    if pocket_id.startswith("corner_"):
        from ..tray.session import corner_label

        return f"{pocket_id} — {corner_label(layout, pocket_id)} of the plate (middle of the rounded corner, top edge)"
    p = layout.pocket(pocket_id)
    rows = [q for q in layout.pockets if q.sku == p.sku]
    n = rows.index(p) + 1
    where = "nearest the arm" if n == 1 else "farthest from the arm" if n == len(rows) else f"row {n} of {len(rows)}"
    return f"{pocket_id} — {names.get(p.sku, p.sku)} column, {where}"


def summarize(cal: TrayCalibration, layout: TrayLayout, pan_xy: np.ndarray | None) -> list[str]:
    """Report lines: fit quality, tilt, warnings, worst grip angle per item type."""
    out = [f"fit error {cal.rms_mm:.1f} mm RMS (" + ", ".join(f"{p.pocket} {p.residual_mm:.1f}" for p in cal.points) + ")",
           f"tray tilt {tilt_deg(cal):.0f}° from flat"]
    o = layout.outline
    if pan_xy is not None:
        center = ((o["xmin"] + o["xmax"]) / 2, (o["ymin"] + o["ymax"]) / 2)
        out += [f"WARNING: {w}" for w in checks(cal, center, pan_xy)]
        worst: dict[str, float] = {}
        for p in layout.pockets:
            worst[p.sku] = max(worst.get(p.sku, 0.0), approach_misalignment_deg(cal, p.x, p.y, pan_xy))
        out.append("worst grip angle (no wrist yaw): " + ", ".join(f"{k} {v:.0f}°" for k, v in worst.items()))
    return out


def run_tray_calibration(arm: Arm, tip_fn: Callable[[dict[str, float]], np.ndarray], tray: TrayCfg,
                         names: dict[str, str], pan_xy: np.ndarray | None = None,
                         input_fn: Callable[[str], str] = input, out: Callable[[str], None] = print,
                         sleep: Callable[[float], None] = time.sleep,
                         hold_target: Callable[[dict[str, float]], dict[str, float]] | None = None
                         ) -> TrayCalibration | None:
    """Interactive calibration. Returns the saved calibration, or None if cancelled / rejected."""
    from ..tray.session import CORNER_ORDER

    layout = tray.layout()
    corners = tray.calibration_mode == "corners"
    refs = CORNER_ORDER if corners else tray.reference_points
    if len(refs) < 3:
        raise CalibrationError("tray.reference_points needs at least 3 pockets")
    for pid in refs:
        layout.point(pid)  # KeyError early for a typo

    out("Tray calibration: put the gripper tip (closed) on " + (
        "each plate CORNER below (middle of the rounded corner, at the top face's edge)." if corners
        else "the CENTER of each pocket below, at the tray face."))
    input_fn("Motors are about to turn OFF — hold the arm, then press Enter...")
    arm.set_torque(False)
    measured: list[tuple[str, tuple[float, float], np.ndarray]] = []
    try:
        while len(measured) < len(refs):
            i = len(measured)
            pid = refs[i]
            ans = input_fn(f"[{i + 1}/{len(refs)}] {describe_pocket(layout, pid, names)}\n"
                           "    tip on its center, then Enter   (b = redo previous, q = quit): ").strip().lower()
            if ans == "q":
                out("cancelled — nothing saved")
                return None
            if ans == "b":
                if measured:
                    measured.pop()
                continue
            reads = []
            for _ in range(SAMPLES):
                reads.append(arm.read())
                sleep(0.02)
            pose = {j: float(np.mean([r[j] for r in reads])) for j in reads[0]}
            tip = np.asarray(tip_fn(pose), float)
            measured.append((pid, layout.point(pid), tip))
            out(f"    {pid}: tip at ({tip[0] * 1000:.1f}, {tip[1] * 1000:.1f}, {tip[2] * 1000:.1f}) mm")

        try:
            cal = calibrate(measured)
        except CalibrationError as e:
            out(f"calibration rejected: {e}")
            return None
        for line in summarize(cal, layout, pan_xy):
            out(line)
        if input_fn(f"Save to {tray.calibration_file}? [Y/n] ").strip().lower() in ("", "y", "yes"):
            cal.save(tray.calibration_file)
            out("saved")
            return cal
        out("not saved")
        return None
    finally:
        out("Hold the arm: motors turning back ON at the current position.")
        pose = arm.read()
        arm.write(hold_target(pose) if hold_target else pose)  # + gravity feed-forward on the real arm
        arm.set_torque(True)
