"""CalibrationSession: the state behind the dashboard's calibration tab (captured points + live fit preview).

Arm-agnostic: the caller measures the tip and passes it in. See `calibration.py` for the math.
"""

from __future__ import annotations

import math

import numpy as np

from .calibration import (MAX_RMS_MM, CalibrationError, TrayCalibration, approach_misalignment_deg, calibrate,
                          checks, tilt_deg)
from .layout import TrayLayout


def corner_label(layout: TrayLayout, pocket_id: str) -> str:
    """'Bottom-left' etc. as seen from the arm: bottom = nearest the arm (low DXF y), left = low DXF x."""
    p, o = layout.pocket(pocket_id), layout.outline
    cx, cy = (o["xmin"] + o["xmax"]) / 2, (o["ymin"] + o["ymax"]) / 2
    return f"{'Bottom' if p.y < cy else 'Top'}-{'left' if p.x < cx else 'right'}"


class CalibrationSession:
    def __init__(self, tray, pan_xy: np.ndarray | None = None):
        self.tray, self.pan_xy = tray, pan_xy
        self.layout: TrayLayout = tray.layout()
        self.active = False
        self.points: dict[str, np.ndarray] = {}  # pocket id -> measured tip (robot frame, m), in capture order

    # ---------- points ----------
    def capture(self, pocket_id: str, tip_m: np.ndarray) -> None:
        self.layout.pocket(pocket_id)  # KeyError for unknown pockets
        self.points.pop(pocket_id, None)
        self.points[pocket_id] = np.asarray(tip_m, float)

    def remove(self, pocket_id: str) -> None:
        self.points.pop(pocket_id, None)

    def clear(self) -> None:
        self.points.clear()
        self.active = False

    # ---------- fit ----------
    def _measured(self):
        return [(pid, (self.layout.pocket(pid).x, self.layout.pocket(pid).y), tip) for pid, tip in self.points.items()]

    def _fit(self) -> tuple[TrayCalibration | None, str | None]:
        if len(self.points) < 3:
            return None, f"capture at least 3 points ({len(self.points)} so far)"
        try:
            cal = calibrate(self._measured(), max_rms_mm=math.inf)
        except CalibrationError as e:
            return None, str(e)
        if cal.rms_mm > MAX_RMS_MM:
            worst = max(cal.points, key=lambda p: p.residual_mm)
            return cal, (f"fit error {cal.rms_mm:.1f} mm RMS is above {MAX_RMS_MM} mm; worst point {worst.pocket} "
                         f"({worst.residual_mm:.1f} mm) — recapture it")
        return cal, None

    def save(self) -> TrayCalibration:
        cal, error = self._fit()
        if cal is None or error:
            raise CalibrationError(error or "no fit")
        cal.save(self.tray.calibration_file)
        return cal

    # ---------- report ----------
    def to_dict(self) -> dict:
        cal, error = self._fit()
        residual = {p.pocket: p.residual_mm for p in cal.points} if cal else {}
        refs = self.tray.reference_points
        fit = None
        if cal is not None or len(self.points) >= 3:
            fit = {"ok": cal is not None and error is None, "error": error,
                   "rms_mm": cal.rms_mm if cal else None, "tilt_deg": round(tilt_deg(cal), 1) if cal else None,
                   "warnings": [], "grip_angles": {}, "matrix": cal.matrix() if cal else None}
            if cal is not None and self.pan_xy is not None:
                o = self.layout.outline
                fit["warnings"] = checks(cal, ((o["xmin"] + o["xmax"]) / 2, (o["ymin"] + o["ymax"]) / 2), self.pan_xy)
                for p in self.layout.pockets:
                    a = approach_misalignment_deg(cal, p.x, p.y, self.pan_xy)
                    fit["grip_angles"][p.sku] = round(max(fit["grip_angles"].get(p.sku, 0.0), a), 1)
        saved = self.tray.calibration()
        return {
            "active": self.active,
            "reference_points": [{"pocket": pid, "label": corner_label(self.layout, pid), "captured": pid in self.points}
                                 for pid in refs],
            "points": [{"pocket": pid, "label": corner_label(self.layout, pid) if pid in refs else None,
                        "tray_mm": [self.layout.pocket(pid).x, self.layout.pocket(pid).y],
                        "robot_m": [round(float(v), 5) for v in tip], "residual_mm": residual.get(pid)}
                       for pid, tip in self.points.items()],
            "fit": fit,
            "saved": None if saved is None else {"created": saved.created, "rms_mm": saved.rms_mm},
        }
