"""Box geometry and the box pose from a hand-guided grasp of its tab.

Box frame (as in box.step, meters): origin = bottom center, z up, the tab sticks out of the −y wall.
Measured from box.step (2026-11-07): body 76.2 × 76.2 × 77 mm, open top, 0.8 mm walls; the tab is a 12.6 mm wide
(x ±6.3) plate along the top edge (z 75–77) from the wall (y −38.1) out to y −61.0, with a rib below it to
z 38.5. The jaws close across the tab's 12.6 mm width with the gripper pointing down.

Placement rule (user, 2026-11-07): a box's tab faces the arm's swivel (pan axis), so the box +y axis points
radially outward. With 5 DOF the tool yaw follows the pan, so a box carried around keeps its tab facing the
swivel automatically.

The grasp is rigid: the tool pose relative to the box (`tool_in_box`) is fixed once the jaws hold the tab. A box
is set down anywhere by moving that grasp (same height → same table contact). `GripCfg` only estimates where
on the tab the tool point sits — used to draw the box, check collisions and aim drops, not for the grasp itself.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

MM = 0.001
BOX_W = 76.2 * MM  # outer, square
BOX_H = 77.0 * MM
WALL = 0.8 * MM
TAB_W = 12.6 * MM  # across the tab (box x) — what the jaws close on
TAB_OUT = (61.0 - 38.1) * MM  # how far the tab sticks out of the wall
HALF = BOX_W / 2


@dataclass
class GripCfg:
    """Where the tool point sits on the tab when the jaws are closed on it (estimate; `cell.yaml` `boxes:`)."""

    from_top_mm: float = 10.0  # tool point below the tab top (= box top)
    from_wall_mm: float = 11.0  # tool point out from the box wall along the tab (tab is 22.9 mm long)
    max_tilt_deg: float = 25.0  # capture refused if the gripper isn't pointing down within this
    max_table_mm: float = 20.0  # capture refused if the box bottom isn't this close to the table (z = 0)


def radial(pan_xy: np.ndarray, p: np.ndarray) -> np.ndarray:
    """Horizontal unit vector from the pan axis out through point p."""
    d = np.asarray(p, float)[:2] - pan_xy
    n = np.linalg.norm(d)
    if n < 1e-6:
        raise ValueError("point is on the pan axis")
    return np.array([d[0] / n, d[1] / n, 0.0])


def box_frame(center_xy, bottom_z: float, pan_xy: np.ndarray) -> np.ndarray:
    """4x4 box pose with its tab facing the swivel: +y radially outward, z up."""
    c = np.array([center_xy[0], center_xy[1], bottom_z], float)
    y = radial(pan_xy, c)
    z = np.array([0.0, 0.0, 1.0])
    T = np.eye(4)
    T[:3, :3] = np.column_stack([np.cross(y, z), y, z])
    T[:3, 3] = c
    return T


class CaptureError(ValueError):
    """The hand-guided pose doesn't look like the jaws holding a box tab (message is shown to the user)."""


def box_from_grasp(T_tool: np.ndarray, pan_xy: np.ndarray, g: GripCfg = GripCfg()) -> tuple[np.ndarray, dict]:
    """Box pose (4x4) from the tool pose with the jaws closed on the tab. Also returns a report.
    The fixed jaw's inner face is at the tool point and the part sits toward tool −x (from the jaw meshes), so
    the tab's center plane is TAB_W/2 toward −x."""
    T_tool = np.asarray(T_tool, float)
    p, x_tool, z_tool = T_tool[:3, 3], T_tool[:3, 0], T_tool[:3, 2]
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(-z_tool[2])))))  # 0 = pointing straight down
    if tilt > g.max_tilt_deg:
        raise CaptureError(f"the gripper should point down at the tab (it's {tilt:.0f}° off vertical)")
    tab_mid = p - x_tool * (TAB_W / 2)
    u = radial(pan_xy, tab_mid)  # tab faces the swivel: the box lies further out
    center_xy = tab_mid[:2] + u[:2] * (HALF + g.from_wall_mm * MM)
    bottom = p[2] + g.from_top_mm * MM - BOX_H
    if abs(bottom) > g.max_table_mm * MM:
        raise CaptureError(f"that puts the box bottom {bottom * 1000:+.0f} mm from the table — are the jaws on the "
                           f"tab, about {g.from_top_mm:.0f} mm below its top?")
    T_box = box_frame(center_xy, bottom, pan_xy)
    across = abs(float(x_tool @ u))  # jaws should close across the tab (tangential), not along it
    return T_box, {"tilt_deg": round(tilt, 1), "bottom_mm": round(bottom * 1000, 1),
                   "jaw_along_tab": round(math.degrees(math.asin(min(1.0, across))), 1)}


def footprint(T_box: np.ndarray, margin: float = 0.0) -> np.ndarray:
    """Box outline (with the tab) in the table plane: 4 corners xy of the body + tab bounding rectangle."""
    hw = HALF + margin
    pts = [(-hw, -hw - TAB_OUT), (hw, -hw - TAB_OUT), (hw, hw), (-hw, hw)]
    R, t = T_box[:2, :2], T_box[:2, 3]
    return np.array([R @ np.array(q) + t for q in pts])


def overlaps(a: np.ndarray, b: np.ndarray) -> bool:
    """Convex polygons (Nx2) intersect? Separating-axis test."""
    for poly in (a, b):
        for i in range(len(poly)):
            e = poly[(i + 1) % len(poly)] - poly[i]
            n = np.array([-e[1], e[0]])
            pa, pb = a @ n, b @ n
            if pa.max() < pb.min() or pb.max() < pa.min():
                return False
    return True


def mirror(T_box: np.ndarray, pan_xy: np.ndarray) -> np.ndarray:
    """The same spot mirrored to the other side of the arm's forward line (y -> -y), tab facing the swivel."""
    c = T_box[:3, 3]
    return box_frame((c[0], 2 * pan_xy[1] - c[1]), c[2], pan_xy)


def obstacles(cfg, box_poses: dict[int, np.ndarray], skip: int | None = None) -> list[tuple[str, np.ndarray]]:
    """Table-plane footprints of the other boxes (at their current spots) and of the calibrated parts tray."""
    out = [(f"box {bid}", footprint(T, margin=0.005)) for bid, T in box_poses.items() if bid != skip]
    tray = getattr(cfg, "tray", None)
    cal = tray.calibration() if tray else None
    if cal:
        o = tray.layout().outline
        corners = [cal.to_robot(x, y)[:2] for x, y in ((o["xmin"], o["ymin"]), (o["xmax"], o["ymin"]),
                                                        (o["xmax"], o["ymax"]), (o["xmin"], o["ymax"]))]
        out.append(("the parts tray", np.array(corners)))
    return out
