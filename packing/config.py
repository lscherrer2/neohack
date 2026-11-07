"""Cell configuration: loaded from cell.yaml into plain dataclasses."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import yaml


@dataclass
class GripperCfg:
    open: float = 60.0  # gripper value when open (0..100)
    closed: float = 0.0  # gripper value when fully closed (empty)
    speed: float = 40.0  # units/s while closing
    squeeze: float = 3.0  # extra closing past the contact point while holding
    min_part_width: float = 3.0  # grip narrower than this (from `closed`) = missed the part
    min_box_width: float = 10.0  # same, for boxes


@dataclass
class MotionCfg:
    speed: float = 30.0  # units/s for free moves
    approach_speed: float = 12.0  # units/s for the last bit down to / up from a grasp
    settle_tol: float = 2.0  # every joint within this of the target counts as "arrived"
    settle_timeout: float = 3.0  # s
    max_following_error: float = 20.0  # stop if a joint lags its command by more (collision/stall)
    max_step: float = 25.0  # library safety cap per command (max_relative_target); > sag_max
    fps: int = 50
    sag_gain: float = 0.08  # sag compensation: fraction of the error learned per tick (0 = off)
    sag_max: float = 15.0  # max correction, units
    lead_s: float = 0.0  # velocity feed-forward: command this many seconds ahead (servo lag ~0.1 s; 0 = off)
    lead_max: float = 10.0  # cap on the feed-forward, units
    gravity_gain: float = 0.0  # gravity feed-forward: units of sag per N*m of model gravity torque (real arm only; 0 = off)
    servo_p: int | None = None  # servo P gain for body joints (connect default 16, factory 32)
    servo_accel: int | None = None  # servo acceleration register (connect default 254 = max)


@dataclass
class BoxSlot:
    pose: str  # location name (poses `<pose>.above` / `<pose>.at`)
    sku: str | None = None  # None = any part


@dataclass
class CellConfig:
    port: str
    robot_id: str
    poses_file: Path
    inventory_db: Path
    travel_pose: str | None
    part_slots: dict[str, str]  # location -> sku
    box_supply: list[str]
    pack_station: str
    box_output: list[str]
    box_slots: list[BoxSlot]
    catalog: dict[str, str] = field(default_factory=dict)  # sku -> spoken product name (voice ordering)
    gripper: GripperCfg = field(default_factory=GripperCfg)
    motion: MotionCfg = field(default_factory=MotionCfg)

    @property
    def locations(self) -> list[str]:
        return [*self.part_slots, *self.box_supply, self.pack_station, *self.box_output,
                *(s.pose for s in self.box_slots)]

    def required_poses(self) -> list[str]:
        names = [f"{loc}.{k}" for loc in self.locations for k in ("above", "at")]
        if self.travel_pose:
            names.insert(0, self.travel_pose)
        return names


def load_config(path: str | Path) -> CellConfig:
    path = Path(path)
    raw = yaml.safe_load(path.read_text())
    base = path.parent

    def rel(p: str) -> Path:
        q = Path(p).expanduser()
        return q if q.is_absolute() else base / q

    r, loc = raw["robot"], raw["locations"]
    return CellConfig(
        port=r["port"],
        robot_id=r.get("id", "my_awesome_follower_arm"),
        poses_file=rel(raw.get("poses_file", "data/poses.json")),
        inventory_db=(Path(":memory:") if raw.get("inventory_db") == ":memory:"
                      else rel(raw.get("inventory_db", "data/inventory.db"))),
        travel_pose=raw.get("travel_pose", "home"),
        part_slots={k: str(v) for k, v in (loc.get("part_slots") or {}).items()},
        box_supply=list(loc.get("box_supply") or []),
        pack_station=loc["pack_station"],
        box_output=list(loc.get("box_output") or []),
        box_slots=[BoxSlot(**s) if isinstance(s, dict) else BoxSlot(pose=s) for s in raw.get("box_slots") or []],
        catalog={str(k): str(v) for k, v in (raw.get("catalog") or {}).items()},
        gripper=GripperCfg(**(raw.get("gripper") or {})),
        motion=MotionCfg(**(raw.get("motion") or {})),
    )
