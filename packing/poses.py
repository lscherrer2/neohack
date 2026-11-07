"""Named joint-space poses, persisted as JSON. Poses hold the 5 body joints (gripper is separate)."""

from __future__ import annotations

import json
from pathlib import Path

from .joints import BODY

DEFAULTS = {"home": dict.fromkeys(BODY, 0.0)}  # calibration mid-pose


class PoseNotTaughtError(KeyError):
    """A pose name was used before it was taught."""


class PoseStore:
    """Taught poses by name, loaded from and saved to one JSON file. `home` always exists."""

    def __init__(self, path: Path):
        self.path = Path(path)
        self._poses: dict[str, dict[str, float]] = {}
        if self.path.exists():
            self._poses = json.loads(self.path.read_text())

    def __contains__(self, name: str) -> bool:
        return name in self._poses or name in DEFAULTS

    def get(self, name: str) -> dict[str, float]:
        if name in self._poses:
            return dict(self._poses[name])
        if name in DEFAULTS:
            return dict(DEFAULTS[name])
        raise PoseNotTaughtError(f"Pose '{name}' not taught yet. Run: python -m packing teach {name}")

    def set(self, name: str, pose: dict[str, float]) -> None:
        self._poses[name] = {j: round(float(pose[j]), 2) for j in BODY}
        self.path.parent.mkdir(parents=True, exist_ok=True)
        tmp = self.path.with_suffix(".tmp")
        tmp.write_text(json.dumps(self._poses, indent=2, sort_keys=True))
        tmp.replace(self.path)  # atomic

    def names(self) -> list[str]:
        return sorted(set(self._poses) | set(DEFAULTS))

    def missing(self, required: list[str]) -> list[str]:
        return [n for n in required if n not in self]
