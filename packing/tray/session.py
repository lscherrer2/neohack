"""CalibrationSession: the state behind the dashboard's calibration tab (captured points + live fit preview).

Arm-agnostic: the caller measures the tip and passes it in. See `calibration.py` for the math.
"""

from __future__ import annotations

import math

import numpy as np

from .calibration import (MAX_RMS_MM, CalibrationError, TrayCalibration, approach_misalignment_deg, calibrate,
                          checks, tilt_deg)
from .layout import TrayLayout


MODES = ("corners", "pockets")
CORNER_ORDER = ["corner_bl", "corner_tl", "corner_br", "corner_tr"]


def corner_label(layout: TrayLayout, point_id: str) -> str:
    """'Bottom-left' etc. as seen from the arm: bottom = nearest the arm (low DXF y), left = low DXF x.
    Plate corners get ' corner' appended."""
    x, y = layout.point(point_id)
    o = layout.outline
    cx, cy = (o["xmin"] + o["xmax"]) / 2, (o["ymin"] + o["ymax"]) / 2
    label = f"{'Bottom' if y < cy else 'Top'}-{'left' if x < cx else 'right'}"
    return f"{label} corner" if point_id.startswith("corner_") else label


class CalibrationSession:
    def __init__(self, tray, pan_xy: np.ndarray | None = None):
        self.tray, self.pan_xy = tray, pan_xy
        self.layout: TrayLayout = tray.layout()
        self.active = False
        self.mode = tray.calibration_mode if tray.calibration_mode in MODES else "corners"
        self.points: dict[str, np.ndarray] = {}  # point id -> measured tip (robot frame, m), in capture order

    @property
    def refs(self) -> list[str]:
        """Reference points for the current mode: the 4 plate corners, or the configured pockets."""
        return CORNER_ORDER if self.mode == "corners" else list(self.tray.reference_points)

    def set_mode(self, mode: str) -> None:
        """Switch corners/pockets. Captured points are kept (any point can be part of the fit)."""
        if mode not in MODES:
            raise ValueError(f"mode must be one of {MODES}")
        self.mode = mode

    # ---------- points ----------
    def capture(self, point_id: str, tip_m: np.ndarray) -> None:
        self.layout.point(point_id)  # KeyError for unknown pockets / corners
        self.points.pop(point_id, None)
        self.points[point_id] = np.asarray(tip_m, float)

    def remove(self, pocket_id: str) -> None:
        self.points.pop(pocket_id, None)

    def clear(self) -> None:
        self.points.clear()
        self.active = False

    # ---------- fit ----------
    def _measured(self):
        return [(pid, self.layout.point(pid), tip) for pid, tip in self.points.items()]

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
        refs = self.refs
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
            "mode": self.mode,
            "reference_points": [{"pocket": pid, "label": corner_label(self.layout, pid), "captured": pid in self.points}
                                 for pid in refs],
            "points": [{"pocket": pid, "label": corner_label(self.layout, pid) if pid in refs else None,
                        "tray_mm": list(self.layout.point(pid)),
                        "robot_m": [round(float(v), 5) for v in tip], "residual_mm": residual.get(pid)}
                       for pid, tip in self.points.items()],
            "fit": fit,
            "saved": None if saved is None else {"created": saved.created, "rms_mm": saved.rms_mm},
        }
