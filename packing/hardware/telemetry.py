"""TelemetryArm: wraps any Arm and publishes the joints it reads (throttled) for the live 3D view.

Motion reads the arm every control tick, so wrapping the arm handed to Motion gives live telemetry during
packing at no extra bus traffic. `publish` must never raise into the control loop: errors are swallowed.
"""

from __future__ import annotations

import logging
import time
from typing import Callable

from .arm import Arm

log = logging.getLogger(__name__)

Publish = Callable[[dict[str, float], bool], None]  # (joints, torque_on)


class TelemetryArm:
    def __init__(self, arm: Arm, publish: Publish, min_interval_s: float = 0.1,
                 clock: Callable[[], float] = time.monotonic):
        self.arm, self.publish, self.min_interval_s, self.clock = arm, publish, min_interval_s, clock
        self.torque = True
        self._last = float("-inf")

    def read(self) -> dict[str, float]:
        joints = self.arm.read()
        now = self.clock()
        if now - self._last >= self.min_interval_s:
            self._last = now
            try:
                self.publish(dict(joints), self.torque)
            except Exception as e:  # noqa: BLE001 — telemetry must not stop the arm
                log.debug("telemetry publish failed: %s", e)
        return joints

    def write(self, pose: dict[str, float]) -> None:
        self.arm.write(pose)

    def set_torque(self, on: bool) -> None:
        self.arm.set_torque(on)
        self.torque = on

    def tune(self, p_gain: int | None = None, accel: int | None = None) -> None:
        self.arm.tune(p_gain=p_gain, accel=accel)

    def close(self, release: bool = False) -> None:
        self.arm.close(release=release)


def http_publisher(server: str, source: str = "remote", timeout_s: float = 0.5) -> Publish:
    """Publish to the shop server's POST /robot/telemetry (for a robot agent in its own process).
    Non-blocking: a background thread sends only the latest value, so a slow server can't stall Motion."""
    import threading

    import httpx

    client = httpx.Client(base_url=server.rstrip("/"), timeout=timeout_s)
    latest: dict = {}
    wake = threading.Event()

    def sender() -> None:
        while True:
            wake.wait()
            wake.clear()
            body = latest.get("body")
            try:
                client.post("/robot/telemetry", json=body)
            except httpx.HTTPError as e:
                log.debug("telemetry post failed: %s", e)

    threading.Thread(target=sender, name="telemetry", daemon=True).start()

    def publish(joints: dict[str, float], torque: bool) -> None:
        latest["body"] = {"joints": joints, "torque": torque, "source": source}
        wake.set()

    return publish
