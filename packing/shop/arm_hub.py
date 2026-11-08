"""ArmHub: the shop server's single owner of the arm (only one process may hold the serial port).

- `connect()` opens the arm (real or simulated) and starts a poll thread (~10 Hz) that reads the joints
  and computes FK link frames for the dashboard's 3D view.
- `borrow()` lends the arm to in-process packing; reads made by Motion keep the view live meanwhile.
- `set_torque()` / `read_tip()` serve the calibration tab (motors off, hand-guided tip capture).
- `external()` accepts telemetry from a robot agent running in another process (it owns the port then).
"""

from __future__ import annotations

import logging
import threading
import time
from contextlib import contextmanager
from typing import Callable, Iterator

import numpy as np

from ..hardware.arm import Arm
from ..hardware.telemetry import TelemetryArm

log = logging.getLogger(__name__)


# Generic poses for the dashboard's preview toggle (calibrated units; checked with FK to look sensible).
PREVIEW_POSES: dict[str, tuple[str, dict[str, float] | None]] = {
    "home": ("Calibration center pose", None),  # None = computed: the URDF zero pose, in units
    "over_tray": ("Over the tray, pointing down", dict(shoulder_pan=0, shoulder_lift=-10, elbow_flex=10,
                                                       wrist_flex=90, wrist_roll=0, gripper=40)),
    "tucked": ("Tucked in", dict(shoulder_pan=0, shoulder_lift=-90, elbow_flex=55, wrist_flex=90,
                                 wrist_roll=0, gripper=0)),
}


class ArmBusy(RuntimeError):
    """The arm can't do that right now (not connected, port error, packing, calibrating)."""


class _Lent(TelemetryArm):
    """Borrowed arm: the borrower's close() must not disconnect the hub's arm."""

    def close(self, release: bool = False) -> None:
        if release:
            self.set_torque(False)


class ArmHub:
    def __init__(self, open_arm: Callable[[], Arm] | None, source: str, model_factory: Callable[[], object] | None = None,
                 poll_s: float = 0.1, stale_after_s: float = 2.0, clock: Callable[[], float] = time.monotonic,
                 tune: dict | None = None, gravity_factory: Callable[[], Callable | None] | None = None):
        """open_arm: opens the arm (None = this process never owns it, e.g. the robot agent runs elsewhere).
        model_factory: returns an ArmModel (FK); None or a failing factory = no 3D frames, joints only.
        tune: servo settings applied on every connect, e.g. {"p_gain": 31, "accel": 254} (cell.yaml motion:).
        gravity_factory: returns the gravity feed-forward (pose -> per-joint offset; real arm only) or None."""
        self._open_arm, self.source = open_arm, source
        self._tune, self._gravity_factory, self._gravity, self._gravity_done = tune or {}, gravity_factory, None, False
        self._model_factory, self._model, self._model_failed = model_factory, None, False
        self.poll_s, self.stale_after_s, self.clock = poll_s, stale_after_s, clock
        self.arm: Arm | None = None
        self.torque: bool | None = None
        self.mode = "idle"  # idle | calibrating | packing | moving
        self.error: str | None = None
        self._lock = threading.RLock()  # serializes bus access
        self._borrowed = False
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._latest: dict | None = None  # {"joints","torque","source","t","frames","tip"}

    # ---------- kinematics ----------
    @property
    def model(self):
        if self._model is None and self._model_factory and not self._model_failed:
            try:
                self._model = self._model_factory()
            except Exception as e:  # noqa: BLE001 — e.g. placo missing: degrade to joints only
                self._model_failed = True
                log.warning("no kinematics for the 3D view: %s", e)
        return self._model

    @property
    def gravity(self) -> Callable | None:
        """Gravity feed-forward for Motion / holds (real arm only), built once; None in sim or if unavailable."""
        if not self._gravity_done:
            self._gravity_done = True
            if self._gravity_factory:
                try:
                    self._gravity = self._gravity_factory()
                except Exception as e:  # noqa: BLE001 — e.g. placo missing: hold without it
                    log.warning("no gravity feed-forward: %s", e)
        return self._gravity

    def hold_target(self, pose: dict[str, float]) -> dict[str, float]:
        """Command that keeps the arm AT `pose`: pose + gravity feed-forward on the body joints (servos are
        P-only, so commanding the measured pose alone lets loaded joints sag a little more)."""
        g = self.gravity
        if not g:
            return dict(pose)
        offset = g(pose)
        return {j: v + offset.get(j, 0.0) for j, v in pose.items()}

    @property
    def pan_xy(self) -> np.ndarray | None:
        return None if self.model is None else self.model.pan_xy

    # ---------- connection ----------
    @property
    def connected(self) -> bool:
        return self.arm is not None

    def connect(self) -> None:
        with self._lock:
            if self.arm is not None:
                return
            if self._open_arm is None:
                raise ArmBusy("this server doesn't own the arm (the robot agent runs in another process)")
            try:
                self.arm = self._open_arm()
                if self._tune:  # same servo stiffness / acceleration as every other arm user (RAM only)
                    self.arm.tune(**self._tune)
            except Exception as e:  # noqa: BLE001 — port busy / unplugged / calibration missing
                if self.arm is not None:
                    self.arm.close(release=False)
                    self.arm = None
                self.error = f"couldn't open the arm: {e}"
                raise ArmBusy(self.error) from e
            self.torque, self.error = True, None
        self._stop.clear()
        self._thread = threading.Thread(target=self._poll, name="arm-hub", daemon=True)
        self._thread.start()

    def disconnect(self) -> None:
        if self.mode != "idle":
            raise ArmBusy(f"arm is {self.mode}")
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2)
        with self._lock:
            if self.arm is not None:
                self.arm.close(release=False)
            self.arm, self.torque = None, None

    close = disconnect

    def _poll(self) -> None:
        while not self._stop.is_set():
            if not self._borrowed:
                try:
                    with self._lock:
                        if self.arm is None:
                            return
                        joints = self.arm.read()
                    self._update(joints, bool(self.torque), self.source)
                except Exception as e:  # noqa: BLE001 — keep polling; show the error
                    self.error = f"arm read failed: {e}"
            self._stop.wait(self.poll_s)

    # ---------- telemetry ----------
    def _update(self, joints: dict[str, float], torque: bool, source: str) -> None:
        frames = tip = None
        model = self.model
        if model is not None:
            try:
                with self._lock:  # the model is shared between threads
                    fr = model.link_frames(joints)
                frames = {k: v.round(5).tolist() for k, v in fr.items()}
                tip = [round(float(v), 5) for v in fr["gripper_frame_link"][:3, 3]]
            except Exception as e:  # noqa: BLE001
                log.debug("FK failed: %s", e)
        self._latest = {"joints": {k: round(float(v), 2) for k, v in joints.items()}, "torque": torque,
                        "source": source, "t": self.clock(), "frames": frames, "tip": tip}

    def external(self, joints: dict[str, float], torque: bool, source: str = "remote") -> None:
        """Telemetry posted by a robot agent in another process. Ignored while this hub owns the arm."""
        if self.arm is None:
            self._update(joints, torque, source)

    def state(self) -> dict:
        s = self._latest
        stale = None if s is None else round(self.clock() - s["t"], 2)
        remote_live = s is not None and s["source"] == "remote" and stale <= self.stale_after_s
        return {
            "connected": self.connected or remote_live,
            "source": self.source if self.connected else ("remote" if remote_live else None),
            "torque": self.torque if self.connected else (s["torque"] if remote_live else None),
            "mode": self.mode if self.connected else ("packing" if remote_live else "idle"),
            "stale_s": stale, "error": self.error,
            "joints": s["joints"] if s else None, "frames": s["frames"] if s else None, "tip": s["tip"] if s else None,
        }

    def preview(self, name: str) -> dict:
        """ArmState for a named generic pose — no arm needed (FK only). KeyError for unknown names."""
        preset = PREVIEW_POSES[name][1]
        if self.model is None:
            raise ArmBusy("kinematics unavailable (placo not installed?)")
        if preset is None:  # the calibration center pose = URDF zero, expressed in calibrated units
            joints = {**self.model.to_units(np.zeros(5)), "gripper": 30.0}
        else:
            joints = dict(preset)
        with self._lock:
            fr = self.model.link_frames(joints)
        return {"connected": False, "source": "preview", "torque": None, "mode": "idle", "stale_s": None,
                "error": None, "joints": joints, "frames": {k: v.round(5).tolist() for k, v in fr.items()},
                "tip": [round(float(v), 5) for v in fr["gripper_frame_link"][:3, 3]]}

    # ---------- packing ----------
    @contextmanager
    def borrow(self, mode: str = "packing", allow: tuple[str, ...] = ("idle",)) -> Iterator[Arm]:
        """Lend the arm (e.g. to a packer for one order) under `mode`; the previous mode comes back after.
        Raises ArmBusy unless the current mode is in `allow` (packing is refused while calibrating)."""
        self.connect()
        with self._lock:
            if self.mode not in allow:
                raise ArmBusy(f"arm is {self.mode}")
            previous, self.mode, self._borrowed = self.mode, mode, True
        lent = _Lent(self.arm, lambda j, t: self._update(j, t, self.source))
        lent.torque = bool(self.torque)
        try:
            yield lent
        finally:
            with self._lock:
                self.mode, self._borrowed = previous, False
                self.torque = lent.torque

    # ---------- calibration ----------
    def set_torque(self, on: bool) -> None:
        with self._lock:
            if self.arm is None:
                raise ArmBusy("arm not connected")
            if on:  # hold where it is now (with gravity feed-forward), then enable
                self.arm.write(self.hold_target(self.arm.read()))
            self.arm.set_torque(on)
            self.torque = on

    def read_tip(self, samples: int = 5, sleep: Callable[[float], None] = time.sleep) -> np.ndarray:
        """Tip position (robot frame, m) averaged over a few reads (hand tremor)."""
        pose = self.read_pose(samples, sleep)
        with self._lock:
            return np.asarray(self.model.tip(pose), float)

    def read_tool(self, samples: int = 5, sleep: Callable[[float], None] = time.sleep) -> tuple[dict, np.ndarray]:
        """(body joint pose, tool pose 4x4 = gripper_frame_link in the robot frame) of the averaged joint reading."""
        pose = self.read_pose(samples, sleep)
        body = {j: v for j, v in pose.items() if j != "gripper"}
        with self._lock:
            return body, np.asarray(self.model.fk(body), float)

    def reload_model(self) -> None:
        """Rebuild the model from its factory (e.g. new joint offsets were saved)."""
        with self._lock:
            self._model, self._model_failed = None, False

    def read_pose(self, samples: int = 5, sleep: Callable[[float], None] = time.sleep) -> dict[str, float]:
        """Joint pose averaged over a few reads (hand tremor)."""
        if self.model is None:
            raise ArmBusy("kinematics unavailable (placo not installed?)")
        reads = []
        for _ in range(samples):
            with self._lock:
                if self.arm is None:
                    raise ArmBusy("arm not connected")
                reads.append(self.arm.read())
            sleep(0.02)
        return {j: float(np.mean([r[j] for r in reads])) for j in reads[0]}
