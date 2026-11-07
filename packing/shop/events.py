"""In-memory event log for the dashboard (thread-safe; newest 500 kept)."""

from __future__ import annotations

import threading
import time
from collections import deque


class EventLog:
    def __init__(self, maxlen: int = 500):
        self._events: deque[dict] = deque(maxlen=maxlen)
        self._seq = 0
        self._lock = threading.Lock()
        self.robot_last_seen: float | None = None

    def record(self, kind: str, text: str, **data) -> dict:
        """kind: tool | robot | order | system | error"""
        with self._lock:
            self._seq += 1
            ev = {"seq": self._seq, "ts": time.time(), "kind": kind, "text": text, "data": data}
            self._events.append(ev)
            return ev

    def since(self, after: int = 0) -> list[dict]:
        with self._lock:
            return [e for e in self._events if e["seq"] > after]

    def robot_seen(self) -> None:
        self.robot_last_seen = time.time()
