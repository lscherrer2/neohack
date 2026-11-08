"""Single worker thread: one arm, one order at a time. queued -> packing -> packed -> calling -> called."""

from __future__ import annotations

import logging
import threading
import time
from typing import Protocol

from .orders import Order, OrderStore

log = logging.getLogger("packing.worker")


class Packer(Protocol):
    def pack(self, order: Order) -> str:
        """Physically pack the order. Returns a box id. Raises on failure."""


class SimPacker:
    """Pretend packing for development: just waits."""

    def __init__(self, seconds_per_item: float = 2.0):
        self.seconds_per_item = seconds_per_item

    def pack(self, order: Order) -> str:
        time.sleep(self.seconds_per_item * len(order.items))
        return f"SIM-BOX-{order.id}"


class Worker:
    def __init__(self, store: OrderStore, packer: Packer, notifier, poll_s: float = 1.0):
        self.store, self.packer, self.notifier, self.poll_s = store, packer, notifier, poll_s
        self._wake = threading.Event()
        self._stop = threading.Event()
        self.thread: threading.Thread | None = None
        self.current: int | None = None
        self.paused = False

    def start(self) -> None:
        if recovered := self.store.recover():
            log.warning("recovered interrupted orders: %s", recovered)
        self.thread = threading.Thread(target=self._run, name="packing-worker", daemon=True)
        self.thread.start()

    def stop(self, timeout: float = 5.0) -> None:
        self._stop.set()
        self._wake.set()
        if self.thread:
            self.thread.join(timeout)

    def wake(self) -> None:
        self._wake.set()

    def _run(self) -> None:
        while not self._stop.is_set():
            if not self.paused and self.step():
                continue
            self._wake.wait(self.poll_s)
            self._wake.clear()

    def step(self) -> bool:
        """Process one order if any. Returns True if it did something."""
        order = self.store.claim_next()
        if not order:
            return False
        self.current = order.id
        log.info("packing order %s: %s", order.id, order.items)
        try:
            box = self.packer.pack(order)
        except Exception as e:  # noqa: BLE001
            log.exception("order %s failed to pack", order.id)
            self.store.update(order.id, "failed", error=str(e))
            self.paused = True  # a physical failure needs a human; stop taking orders
            self.current = None
            return True
        order = self.store.update(order.id, "packed", box_id=box)
        self.call_back(order)
        self.current = None
        return True

    def call_back(self, order: Order) -> None:
        self.store.update(order.id, "calling")
        try:
            call_id = self.notifier.notify(order)
            self.store.update(order.id, "called", callback_call_id=call_id)
        except Exception as e:  # noqa: BLE001
            log.exception("callback for order %s failed", order.id)
            self.store.update(order.id, "call_failed", error=str(e))
