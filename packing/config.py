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
    # opening just wide enough for a part (packing/tray/pick.py): width + margin, converted to gripper units
    open_margin_mm: float = 10.0  # total extra opening beyond the part width (both sides together)
    jaw_length_mm: float = 80.0  # jaw pivot -> fingertip (URDF estimate; measure to refine)
    deg_per_unit: float = 1.27  # jaw degrees per gripper unit (load_config fills it from the calibration range)

    def units_for(self, width_mm: float) -> float:
        """Gripper units from closed at which the fingertips are `width_mm` apart (chord model, see open_for)."""
        import math

        return math.degrees(2 * math.asin(min(1.0, width_mm / (2 * self.jaw_length_mm)))) / self.deg_per_unit

    def open_for(self, width_mm: float) -> float:
        """Gripper value that opens the fingertips `width_mm + open_margin_mm` apart (capped at `open`).
        Fingertip gap ~ chord 2 L sin(angle / 2); unit 0 = closed (jaws touching)."""
        import math

        half = min(1.0, (width_mm + self.open_margin_mm) / (2 * self.jaw_length_mm))
        units = math.degrees(2 * math.asin(half)) / self.deg_per_unit
        direction = 1.0 if self.open >= self.closed else -1.0
        value = self.closed + direction * units
        return min(value, self.open) if direction > 0 else max(value, self.open)


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
class TrayCfg:
    """Parts tray: pockets come from a DXF; its pose comes from `python -m packing tray-calibrate`."""

    dxf: Path
    calibration_file: Path
    types: list[TrayType]
    reference_points: list[str]  # pockets touched in "pockets" calibration mode (>= 3, spread out)
    calibration_mode: str = "corners"  # corners: touch the 4 plate corners | pockets: touch reference_points
    protrusion_mm: float = 10.0  # how far every item sticks out of the tray face
    extract_mm: float = 120.0  # (unused by picks now: the pull-out is per part — tray/pick.py PickSettings.for_type)
    approach_mm: float = 30.0  # stand-off above the item top before the final straight approach
    # picking (packing/tray/pick.py)
    grasp_standoff_mm: float = 3.81  # tool tip off the tray face when the gripper closes (0.15 in)
    pull_mm: float = 150.0  # MAX straight pull-out along the normal (per part: what clears its socket + up to 40 mm)
    lift_mm: float = 100.0  # then straight up this far
    max_pitch_deg: float = 40.0  # tool may pitch down up to this to fit the pull-out (smallest that fits wins)
    pitch_step_deg: float = 10.0
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
class BoxesCfg:
    """Packing boxes (packing/boxes/): calibration grip estimate, pack spot, carry motion."""

    grip_from_wall_mm: float = 8.0  # tool point out from the box wall along the tab (jaws closed up/down on it)
    pack_spot_mm: tuple[float, float] = (355.0, 0.0)  # box center (robot x, y) where orders are packed. The level
    # tab grip (gripper pointing away along the tab) only reaches with the grip point >= ~25 cm from the swivel
    # -> box centers >= ~30 cm out. 355 works for grips 8–20 mm from the wall (345 failed deeper grips' back-out;
    # 365+ loses the drops over the box)
    carry_mm: float = 30.0  # lift the box straight up this far before moving it (no dragging, clears neighbours)
    fixed_jaw: str = "bottom"  # grip with the FIXED jaw under the tab plate (moving jaw on top) whatever the
    # calibration used — wrist roll ~0; fixed jaw on top needs roll ~180°, at its limit. "top" / "as_calibrated"
    min_center_mm: float = 305.0  # box center at least this far from the swivel: closer, the level approach /
    # back-out along the tab runs into the arm's reach limit (sim: 304 ok, 302 fails)
    approach_mm: float = 22.0  # open jaws come in level along the tab from this far back (and back out after):
    # clears the tab tip (~15 mm behind the grip point + jaw tips); longer runs below the ~25 cm level-grip limit
    line_speed_mm_s: float = 60.0  # straight lifts / slides along the tab (user: 2–3x faster, was 25)
    down_speed_mm_s: float = 30.0  # the last bit down onto the tab / the table (was 12)
    carry_speed_mm_s: float = 120.0  # guided carries (arc up + swing + arc down, one move): max tool-point speed
    carry_joint_speed: float = 60.0  # ...and paced by the busiest joint (units/s) — so a swing close to the swivel
    # isn't held to a slow tool speed
    jaw_gap_mm: float = 24.0  # fingertip gap while approaching / releasing a box (2x the plate + margin's 12 mm)
    free_speed: float = 30.0  # free joint moves in box moves, units/s (was half the motion speed: 15)


@dataclass
class CellConfig:
    port: str
    robot_id: str
    poses_file: Path
    inventory_db: Path
    travel_pose: str | None  # taught pose `home` / `goto` use (None = all-zero mid pose)
    max_items_per_order: int = 10  # items in one order (= one box)
    catalog: dict[str, str] = field(default_factory=dict)  # sku -> spoken product name (voice ordering)
    gripper: GripperCfg = field(default_factory=GripperCfg)
    motion: MotionCfg = field(default_factory=MotionCfg)
    tray: TrayCfg | None = None
    boxes: BoxesCfg = field(default_factory=BoxesCfg)
    desk_file: Path | None = None  # desk plane calibration (packing/boxes/desk.py); sim runs use sim_desk_cal.json
    joint_offsets_file: Path | None = None  # kinematic zero offsets from the desk-touch fit (motion/joint_offsets.py)

    def arm_model(self):
        """ArmModel with this arm's calibration and saved joint offsets (re-read every call)."""
        from .motion.kinematics import arm_model

        return arm_model(self.robot_id, self.joint_offsets_file)

    def desk_plane(self):
        """The saved desk plane, or None (re-read every call: the dashboard may have just saved one)."""
        from .boxes.desk import DeskPlane

        return DeskPlane.load(self.desk_file) if self.desk_file else None

    @property
    def stock_slots(self) -> dict[str, str]:
        """Every place a part can sit, location -> sku: the tray pockets."""
        return self.tray.layout().slots() if self.tray else {}


def load_config(path: str | Path) -> CellConfig:
    path = Path(path)
    raw = yaml.safe_load(path.read_text())
    base = path.parent

    def rel(p: str) -> Path:
        q = Path(p).expanduser()
        return q if q.is_absolute() else base / q

    r = raw["robot"]
    cfg = CellConfig(
        port=r["port"],
        robot_id=r.get("id", "my_awesome_follower_arm"),
        poses_file=rel(raw.get("poses_file", "data/poses.json")),
        inventory_db=(Path(":memory:") if raw.get("inventory_db") == ":memory:"
                      else rel(raw.get("inventory_db", "data/inventory.db"))),
        travel_pose=raw.get("travel_pose", "home"),
        max_items_per_order=int(raw.get("max_items_per_order", 10)),
        catalog={str(k): str(v) for k, v in (raw.get("catalog") or {}).items()},
        gripper=GripperCfg(**(raw.get("gripper") or {})),
        motion=MotionCfg(**(raw.get("motion") or {})),
        tray=_load_tray(raw.get("tray"), rel),
        boxes=_load_boxes(raw.get("boxes")),
        desk_file=rel((raw.get("desk") or {}).get("calibration_file", "data/desk_cal.json")),
    )
    cfg.joint_offsets_file = rel((raw.get("kinematics") or {}).get(
        "offsets_file", f"calibration/{cfg.robot_id}_joint_offsets.json"))
    try:  # jaw degrees per gripper unit from the calibrated range (0..100 spans range_min..range_max)
        from .hardware.so101 import load_calibration

        g = load_calibration(cfg.robot_id)["gripper"]
        cfg.gripper.deg_per_unit = abs(g.range_max - g.range_min) * 360 / 4096 / 100
    except Exception:  # noqa: BLE001 — no calibration file: keep the default
        pass
    if cfg.tray:  # tray item names fill in the catalog
        for t in cfg.tray.types:
            cfg.catalog.setdefault(t.sku, t.name)
    return cfg


CALIBRATION_MODES = ("corners", "pockets")


def _mode(mode: str) -> str:
    if mode not in CALIBRATION_MODES:
        raise ValueError(f"tray.calibration_mode must be one of {CALIBRATION_MODES}, not {mode!r}")
    return mode


def _load_boxes(raw: dict | None) -> BoxesCfg:
    raw = dict(raw or {})
    if "pack_spot_mm" in raw:
        raw["pack_spot_mm"] = tuple(float(v) for v in raw["pack_spot_mm"])
    return BoxesCfg(**raw)


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
        grasp_standoff_mm=float(raw.get("grasp_standoff_mm", 3.81)),
        pull_mm=float(raw.get("pull_mm", 150.0)),
        lift_mm=float(raw.get("lift_mm", 100.0)),
        max_pitch_deg=float(raw.get("max_pitch_deg", 40.0)),
        pitch_step_deg=float(raw.get("pitch_step_deg", 10.0)),
    )
