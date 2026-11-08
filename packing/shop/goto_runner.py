"""Runs a planned "go to" preview move (packing/tray/goto.py) on the hub's arm in a background thread.

Safety: the plan is fully validated before the thread starts; the move is slow (half the configured joint
speed for the free move, ~20 mm/s on the straight line); Motion's following-error check stops on collisions;
`stop()` halts at the next control tick and holds position. If the motors were off (calibrating), they are
turned on at the current pose first — the caller's confirmation must say so.
"""

from __future__ import annotations

import logging
import threading
import time
from typing import Callable

from ..motion.controller import Motion, MotionError
from ..poses import PoseStore
from ..tray.goto import GotoPlan

log = logging.getLogger(__name__)
LINE_SPEED_M_S = 0.02


class Stopped(MotionError):
    """The user pressed Stop."""


class _Stoppable:
    """Arm wrapper: once `stop` is set, the next write raises Stopped (Motion then never sends another target)."""

    def __init__(self, arm, stop: threading.Event):
        self.arm, self.stop = arm, stop

    def read(self):
        return self.arm.read()

    def write(self, pose):
        if self.stop.is_set():
            raise Stopped("stopped by the user")
        self.arm.write(pose)

    def set_torque(self, on: bool) -> None:
        self.arm.set_torque(on)

    def tune(self, p_gain=None, accel=None) -> None:
        self.arm.tune(p_gain=p_gain, accel=accel)

    def close(self, release: bool = False) -> None:
        self.arm.close(release=release)


class GotoRunner:
    def __init__(self, hub, cfg, record: Callable[..., None], sleep: Callable[[float], None] = time.sleep):
        """cfg: CellConfig (motion + gripper settings, poses file). record(kind, text, **data): event log."""
        self.hub, self.cfg, self.record, self.sleep = hub, cfg, record, sleep
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self.last: dict | None = None  # {"plan", "status": running|done|stopped|failed, "error"}

    @property
    def running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def start(self, plan: GotoPlan) -> None:
        if self.running:
            raise RuntimeError("a move is already running")
        self._stop.clear()
        self.last = {"plan": plan.summary(), "status": "running", "error": None}
        self._thread = threading.Thread(target=self._run, args=(plan,), name="arm-goto", daemon=True)
        self._thread.start()

    def stop(self) -> bool:
        if not self.running:
            return False
        self._stop.set()
        return True

    def join(self, timeout: float | None = None) -> None:
        if self._thread:
            self._thread.join(timeout)

    def _run(self, plan: GotoPlan) -> None:
        name = f"{plan.point} (+{plan.standoff_mm:.1f} mm along the normal)"
        try:
            with self.hub.borrow(mode="moving", allow=("idle", "calibrating")) as arm:
                if not arm.torque:  # motors were off for calibration: hold where it is first
                    arm.write(self.hub.hold_target(arm.read()))
                    arm.set_torque(True)
                    self.record("robot", "Motors ON (holding) for a go-to preview")
                stoppable = _Stoppable(arm, self._stop)
                # same tuning as packing: the hub applies servo_p / servo_accel on connect and owns the gravity
                # feed-forward (real arm only); Motion adds velocity feed-forward + at-rest sag learning itself
                motion = Motion(stoppable, PoseStore(self.cfg.poses_file), self.cfg.motion, self.cfg.gripper,
                                sleep=self.sleep, log=log.info, gravity=getattr(self.hub, "gravity", None))
                self.record("robot", f"Moving to {name}", point=plan.point)
                motion.move_to(plan.approach_pose, speed=self.cfg.motion.speed / 2)
                motion.follow(plan.path, max(1.0, plan.line_mm / 1000 / LINE_SPEED_M_S), name)
            self.last["status"] = "done"
            self.record("robot", f"Arrived at {name}; holding", point=plan.point, tilt_deg=round(plan.tilt_deg, 1))
        except Stopped:
            self._hold()
            self.last["status"] = "stopped"
            self.record("robot", f"Go-to {plan.point} stopped by the user; holding position", point=plan.point)
        except Exception as e:  # noqa: BLE001 — MotionError, ArmBusy, bus errors: report, hold
            self._hold()
            self.last.update(status="failed", error=str(e))
            self.record("error", f"Go-to {plan.point} failed: {e}", point=plan.point)
            log.exception("go-to failed")

    def _hold(self) -> None:
        try:
            with self.hub._lock:
                arm = self.hub.arm
                if arm is not None:
                    arm.write(self.hub.hold_target(arm.read()))
        except Exception:  # noqa: BLE001
            pass
