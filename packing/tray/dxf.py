"""Read the tray DXF with ezdxf: round holes (CIRCLE) and closed 4-sided rectangular loops.

Rectangles may be drawn as 4 LINEs or as a polyline; ezdxf's edgeminer finds the closed loops. The
largest loop is the plate outline. Units are whatever the drawing uses (the tray is millimeters).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path

import ezdxf
import ezdxf.bbox
import numpy as np
from ezdxf import edgeminer, edgesmith
from ezdxf.disassemble import recursive_decompose


@dataclass(frozen=True)
class Circle:
    x: float
    y: float
    r: float


@dataclass(frozen=True)
class Rect:
    x: float  # center
    y: float
    w: float  # side along angle_deg
    h: float
    angle_deg: float  # direction of the `w` side, [0, 180)


@dataclass
class Features:
    circles: list[Circle] = field(default_factory=list)
    rects: list[Rect] = field(default_factory=list)
    outline: dict = field(default_factory=dict)  # {"xmin","ymin","xmax","ymax","corner_r"}


def _rect(points: list[tuple[float, float]]) -> Rect | None:
    """4 corner points in loop order -> Rect, or None if the angles aren't right angles."""
    p = np.array(points, float)
    sides = np.roll(p, -1, axis=0) - p
    lens = np.linalg.norm(sides, axis=1)
    if lens.min() < 1e-6:
        return None
    u = sides / lens[:, None]
    if any(abs(u[i] @ u[(i + 1) % 4]) > 1e-3 for i in range(4)):
        return None
    c = p.mean(axis=0)
    return Rect(float(c[0]), float(c[1]), float((lens[0] + lens[2]) / 2), float((lens[1] + lens[3]) / 2),
                math.degrees(math.atan2(u[0][1], u[0][0])) % 180)


def read_features(path: str | Path) -> Features:
    doc = ezdxf.readfile(str(path))
    entities = []
    for e in recursive_decompose(doc.modelspace()):  # explodes blocks/inserts
        if e.dxftype() in ("LWPOLYLINE", "POLYLINE"):
            entities += list(e.virtual_entities())  # -> LINE / ARC segments for the loop finder
        else:
            entities.append(e)
    feats = Features(circles=[Circle(e.dxf.center.x, e.dxf.center.y, e.dxf.radius)
                              for e in entities if e.dxftype() == "CIRCLE"])

    edges = list(edgesmith.edges_from_entities_2d(e for e in entities if e.dxftype() != "CIRCLE"))
    loops = edgeminer.find_all_loops(edgeminer.Deposit(edges)) if edges else []
    if not loops:
        raise ValueError(f"{path}: no closed outline found")
    corners = [[(e.start.x, e.start.y) for e in loop] for loop in loops]

    def area(pts):
        xs, ys = [q[0] for q in pts], [q[1] for q in pts]
        return (max(xs) - min(xs)) * (max(ys) - min(ys))

    outline_i = max(range(len(loops)), key=lambda i: area(corners[i]))
    for i, (loop, pts) in enumerate(zip(loops, corners)):
        straight = all(abs(e.length - e.start.distance(e.end)) < 1e-6 for e in loop)  # no arcs
        if i != outline_i and len(loop) == 4 and straight:
            if r := _rect(pts):
                feats.rects.append(r)
    arcs = [e.payload for e in loops[outline_i] if e.payload.dxftype() == "ARC"]
    box = ezdxf.bbox.extents(entities)
    lo, hi = box.extmin, box.extmax
    feats.outline = {"xmin": round(lo.x, 3), "ymin": round(lo.y, 3), "xmax": round(hi.x, 3), "ymax": round(hi.y, 3),
                     "corner_r": round(max((a.dxf.radius for a in arcs), default=0.0), 3)}
    return feats
