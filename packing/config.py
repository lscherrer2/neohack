"""Cell configuration: loaded from cell.yaml into plain dataclasses."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import yaml

from .tray.layout import TrayLayout, TrayType


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
class TrayCfg:
    """Parts tray: pockets come from a DXF; its pose comes from `python -m packing tray-calibrate`."""

    dxf: Path
    calibration_file: Path
    types: list[TrayType]
    reference_points: list[str]  # pockets touched in "pockets" calibration mode (>= 3, spread out)
    calibration_mode: str = "corners"  # corners: touch the 4 plate corners | pockets: touch reference_points
    protrusion_mm: float = 10.0  # how far every item sticks out of the tray face
    extract_mm: float = 120.0  # pull straight out along the tray normal at least this far
    approach_mm: float = 30.0  # stand-off above the item top before the final straight approach
    _layout: TrayLayout | None = field(default=None, repr=False, compare=False)

    def layout(self) -> TrayLayout:
        """Pockets parsed from the DXF (cached)."""
        if self._layout is None:
            from .tray.dxf import read_features
            from .tray.layout import build_layout

            self._layout = build_layout(read_features(self.dxf), self.types)
        return self._layout

    def calibration(self):
        """Current saved calibration (re-read every call: tray-calibrate may have updated it), or None."""
        from .tray.calibration import TrayCalibration

        return TrayCalibration.load(self.calibration_file)


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
    tray: TrayCfg | None = None

    @property
    def stock_slots(self) -> dict[str, str]:
        """Every place a part can sit, location -> sku: taught part slots plus tray pockets."""
        return {**self.part_slots, **(self.tray.layout().slots() if self.tray else {})}

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
    cfg = CellConfig(
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
        tray=_load_tray(raw.get("tray"), rel),
    )
    if cfg.tray:  # tray item names fill in the catalog
        for t in cfg.tray.types:
            cfg.catalog.setdefault(t.sku, t.name)
    return cfg


CALIBRATION_MODES = ("corners", "pockets")


def _mode(mode: str) -> str:
    if mode not in CALIBRATION_MODES:
        raise ValueError(f"tray.calibration_mode must be one of {CALIBRATION_MODES}, not {mode!r}")
    return mode


def _load_tray(raw: dict | None, rel) -> TrayCfg | None:
    if not raw:
        return None
    types = [TrayType(sku=str(sku), **spec) for sku, spec in (raw.get("types") or {}).items()]
    if not types:
        raise ValueError("tray: needs at least one entry under types")
    return TrayCfg(
        dxf=rel(raw["dxf"]),
        calibration_file=rel(raw.get("calibration_file", "data/tray_cal.json")),
        types=types,
        reference_points=list(raw.get("reference_points") or []),
        calibration_mode=_mode(raw.get("calibration_mode", "corners")),
        protrusion_mm=float(raw.get("protrusion_mm", 10.0)),
        extract_mm=float(raw.get("extract_mm", 120.0)),
        approach_mm=float(raw.get("approach_mm", 30.0)),
    )
