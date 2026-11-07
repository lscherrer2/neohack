"""Tray layout: find the pockets in the tray DXF and name them by item type.

Pocket features (sizes in mm, tolerance `TOL_MM`):
  circle    a round hole of diameter size[0]                 -> pick point = center
  rect      a closed 4-line rectangle size[0] x size[1]       -> pick point = center
            (matches either orientation; yaw_deg = 90 if rotated)
  pin_pair  two holes of diameter size[0], `spacing` apart    -> pick point = midpoint
Pockets of each type are numbered by row: `<sku>_1` is the lowest y (nearest the arm), ties by x.

Tray frame: DXF x/y in the tray face, z out of the face (toward the arm / ceiling), origin = DXF origin.
"""

from __future__ import annotations

import math
from collections import defaultdict
from dataclasses import dataclass, field

from .dxf import Features

TOL_MM = 0.3
FEATURES = ("circle", "rect", "pin_pair")


@dataclass
class TrayType:
    """One item type in the tray (from `tray.types` in cell.yaml)."""

    sku: str
    name: str  # spoken / displayed product name
    feature: str  # circle | rect | pin_pair
    size: list[float]  # circle: [diameter]; rect: [w, h]; pin_pair: [pin diameter]
    spacing: float | None = None  # pin_pair: center distance between the two pins
    grip_axis: str = "x"  # tray-frame axis the gripper fingers close along (before pocket yaw)

    def __post_init__(self) -> None:
        if self.feature not in FEATURES:
            raise ValueError(f"tray type {self.sku}: feature must be one of {FEATURES}, not {self.feature!r}")
        if self.grip_axis not in ("x", "y"):
            raise ValueError(f"tray type {self.sku}: grip_axis must be x or y")
        need = {"circle": 1, "rect": 2, "pin_pair": 1}[self.feature]
        self.size = [float(v) for v in (self.size if isinstance(self.size, list) else [self.size])]
        if len(self.size) != need:
            raise ValueError(f"tray type {self.sku}: {self.feature} needs size with {need} value(s)")
        if self.feature == "pin_pair" and not self.spacing:
            raise ValueError(f"tray type {self.sku}: pin_pair needs spacing")


@dataclass
class Pocket:
    id: str
    sku: str
    x: float  # mm, tray frame
    y: float
    yaw_deg: float = 0.0  # rotation of the feature vs its configured orientation
    shape: dict = field(default_factory=dict)  # for drawing: {"kind": circle|rect|pins, ...}

    def grip_dir(self, grip_axis: str) -> tuple[float, float]:
        """Unit vector (tray frame) the fingers close along."""
        base = 0.0 if grip_axis == "x" else 90.0
        a = math.radians(base + self.yaw_deg)
        return math.cos(a), math.sin(a)


@dataclass
class TrayLayout:
    pockets: list[Pocket]
    outline: dict  # {"xmin","ymin","xmax","ymax","corner_r"} mm
    unmatched: list[str]  # features found in the drawing that no tray type claims

    def pocket(self, pocket_id: str) -> Pocket:
        for p in self.pockets:
            if p.id == pocket_id:
                return p
        raise KeyError(f"no pocket {pocket_id!r} (have {', '.join(p.id for p in self.pockets)})")

    def slots(self) -> dict[str, str]:
        """pocket id -> sku (the inventory's part slots)."""
        return {p.id: p.sku for p in self.pockets}

    def corners(self) -> dict[str, tuple[float, float]]:
        """Plate corner reference points (mm): the middle of each rounded corner, on the plate edge.
        (The sharp corner itself is in mid-air when the corners are rounded.) Bottom = nearest the arm."""
        o, r = self.outline, self.outline.get("corner_r", 0.0)
        k = r * (1 - 1 / math.sqrt(2))  # inset of the arc midpoint from the sharp corner, along x and y
        x0, x1, y0, y1 = o["xmin"] + k, o["xmax"] - k, o["ymin"] + k, o["ymax"] - k
        return {"corner_bl": (round(x0, 3), round(y0, 3)), "corner_tl": (round(x0, 3), round(y1, 3)),
                "corner_br": (round(x1, 3), round(y0, 3)), "corner_tr": (round(x1, 3), round(y1, 3))}

    def point(self, point_id: str) -> tuple[float, float]:
        """Tray-frame (x, y) mm of a calibration reference point: a pocket id or a corner id."""
        corners = self.corners()
        if point_id in corners:
            return corners[point_id]
        p = self.pocket(point_id)
        return p.x, p.y


def _near(a: float, b: float) -> bool:
    return abs(a - b) <= TOL_MM


def build_layout(feats: Features, types: list[TrayType]) -> TrayLayout:
    rects, circles = list(feats.rects), list(feats.circles)
    found: dict[str, list[Pocket]] = defaultdict(list)

    # pin pairs first, so their small holes aren't left over
    for t in (t for t in types if t.feature == "pin_pair"):
        pins = [c for c in circles if _near(2 * c.r, t.size[0])]
        taken: set[int] = set()
        for i, a in enumerate(pins):
            if i in taken:
                continue
            for j in range(i + 1, len(pins)):
                b = pins[j]
                if j not in taken and _near(math.hypot(b.x - a.x, b.y - a.y), t.spacing):
                    taken |= {i, j}
                    yaw = math.degrees(math.atan2(b.y - a.y, b.x - a.x)) % 180
                    found[t.sku].append(Pocket("", t.sku, (a.x + b.x) / 2, (a.y + b.y) / 2, yaw,
                                               {"kind": "pins", "d": t.size[0], "spacing": t.spacing}))
                    break
        claimed = {id(pins[i]) for i in taken}
        circles = [c for c in circles if id(c) not in claimed]

    for t in (t for t in types if t.feature == "circle"):
        hit = [c for c in circles if _near(2 * c.r, t.size[0])]
        found[t.sku] += [Pocket("", t.sku, c.x, c.y, 0.0, {"kind": "circle", "d": t.size[0]}) for c in hit]
        circles = [c for c in circles if c not in hit]

    for t in (t for t in types if t.feature == "rect"):
        w, h = t.size
        hit = []
        for r in rects:
            if _near(r.w, w) and _near(r.h, h):
                yaw = r.angle_deg
            elif _near(r.w, h) and _near(r.h, w):
                yaw = (r.angle_deg + 90) % 180
            else:
                continue
            hit.append(r)
            found[t.sku].append(Pocket("", t.sku, r.x, r.y, yaw, {"kind": "rect", "w": w, "h": h}))
        rects = [r for r in rects if r not in hit]

    pockets: list[Pocket] = []
    for t in types:
        for n, p in enumerate(sorted(found[t.sku], key=lambda p: (round(p.y, 2), round(p.x, 2))), 1):
            p.id = f"{t.sku}_{n}"
            p.x, p.y = round(p.x, 4) + 0.0, round(p.y, 4) + 0.0  # + 0.0 turns -0.0 into 0.0
            p.yaw_deg = round(p.yaw_deg, 3) % 180.0 + 0.0
            pockets.append(p)

    unmatched = [f"circle d={2 * c.r:.2f} at ({c.x:.2f}, {c.y:.2f})" for c in circles]
    unmatched += [f"rect {r.w:.2f}x{r.h:.2f} at ({r.x:.2f}, {r.y:.2f})" for r in rects]
    outline = dict(feats.outline)
    return TrayLayout(pockets, outline, unmatched)

