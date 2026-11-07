"""Raw-terminal keyboard input for jogging the arm (teach pendant)."""

from __future__ import annotations

import select
import sys
import termios
import tty
from contextlib import contextmanager
from typing import Iterator

# key -> (joint, direction). Two keys per joint: first = positive, second = negative.
JOG_KEYS: dict[str, tuple[str, int]] = {
    "q": ("shoulder_pan", +1), "a": ("shoulder_pan", -1),
    "w": ("shoulder_lift", +1), "s": ("shoulder_lift", -1),
    "e": ("elbow_flex", +1), "d": ("elbow_flex", -1),
    "r": ("wrist_flex", +1), "f": ("wrist_flex", -1),
    "t": ("wrist_roll", +1), "g": ("wrist_roll", -1),
    "y": ("gripper", +1), "h": ("gripper", -1),
}

JOG_HELP = "q/a pan  w/s shoulder  e/d elbow  r/f wrist_flex  t/g wrist_roll  y/h gripper"


@contextmanager
def raw_terminal() -> Iterator[None]:
    """Read single keypresses without waiting for Enter; restores the terminal on exit."""
    fd = sys.stdin.fileno()
    saved = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        yield
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, saved)


def read_key(timeout: float) -> str | None:
    """Return the next keypress, or None if none arrives within `timeout` seconds.
    Only valid inside `raw_terminal()`."""
    if select.select([sys.stdin], [], [], timeout)[0]:
        return sys.stdin.read(1)
    return None
