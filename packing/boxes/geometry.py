"""Box geometry and the box pose from a hand-guided grasp of its tab.

Box frame (as in box.step, meters): origin = bottom center, z up, the tab sticks out of the −y wall.
Measured from box.step (2026-11-07): body 76.2 × 76.2 × 77 mm, open top, 0.8 mm walls; the tab is a 12.6 mm wide
(x ±6.3), 2 mm thick plate parallel to the desk along the top edge (z 75–77), from the wall (y −38.1) out to
y −61.0.

Grip (user, 2026-11-07): the gripper is HORIZONTAL — pointing away from the arm along the tab, at tab height —
and the jaws close UP/DOWN on the 2 mm plate. It comes in level along the tab, grips, lifts straight up.

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
TAB_W = 12.6 * MM  # across the tab (box x)
TAB_T = 2.0 * MM  # plate thickness (box z 75..77) — what the jaws close on
TAB_OUT = (61.0 - 38.1) * MM  # how far the tab sticks out of the wall
HALF = BOX_W / 2


@dataclass
class GripCfg:
    """Where the tool point sits on the tab when the jaws are closed on it (estimate; `cell.yaml` `boxes:`)."""

    from_wall_mm: float = 8.0  # tool point out from the box wall along the tab (tab is 22.9 mm long)
    max_tilt_deg: float = 25.0  # capture refused if the gripper isn't level within this
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
    """Box pose (4x4) from the tool pose with the jaws closed (up/down) on the tab, gripper level and pointing
    away from the arm. Also returns a report. The fixed jaw's inner face is at the tool point and the part sits
    toward tool −x (jaw meshes), so the plate's mid-plane is TAB_T/2 toward −x (x is vertical: up or down)."""
    T_tool = np.asarray(T_tool, float)
    p, x_tool, z_tool = T_tool[:3, 3], T_tool[:3, 0], T_tool[:3, 2]
    tilt = math.degrees(math.asin(min(1.0, abs(float(z_tool[2])))))  # 0 = gripper level
    if tilt > g.max_tilt_deg:
        raise CaptureError(f"the gripper should be level, pointing along the tab (it's {tilt:.0f}° off level)")
    jaws = math.degrees(math.acos(min(1.0, abs(float(x_tool[2])))))  # 0 = jaws close straight up/down
    if jaws > 35:
        raise CaptureError(f"the jaws should close up/down on the tab (they're {jaws:.0f}° off)")
    tab_mid = p - x_tool * (TAB_T / 2)
    u = radial(pan_xy, tab_mid)  # tab faces the swivel: the box lies further out
    if float(z_tool[:2] @ u[:2]) < math.cos(math.radians(50)):
        raise CaptureError("the gripper should point away from the arm, along the tab toward the box")
    center_xy = tab_mid[:2] + u[:2] * (HALF + g.from_wall_mm * MM)
    bottom = tab_mid[2] - (BOX_H - TAB_T / 2)
    if abs(bottom) > g.max_table_mm * MM:
        raise CaptureError(f"that puts the box bottom {bottom * 1000:+.0f} mm from the table — are the jaws closed "
                           f"on the tab (the plate along the box's top edge)?")
    T_box = box_frame(center_xy, bottom, pan_xy)
    return T_box, {"tilt_deg": round(tilt, 1), "jaws_off_vertical_deg": round(jaws, 1),
                   "bottom_mm": round(bottom * 1000, 1)}


def box_tilt_deg(T_tool: np.ndarray, tool_in_box: np.ndarray) -> float:
    """How far the box's up axis is from vertical when the tool is at T_tool holding it with this grasp."""
    z = (T_tool @ np.linalg.inv(tool_in_box))[:3, 2]
    return math.degrees(math.acos(max(-1.0, min(1.0, float(z[2])))))


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


# ---------------------------------------------------------------- 3D collision blocks
@dataclass
class Block:
    """An oriented 3D block: 4x4 pose of its center + half extents (m). For collision checks."""

    name: str
    T: np.ndarray
    half: np.ndarray


def blocks_intersect(a: Block, b: Block, margin: float = 0.0) -> bool:
    """Do two oriented blocks (each grown by `margin`) overlap? Separating-axis test (15 axes)."""
    Ra, Rb = a.T[:3, :3], b.T[:3, :3]
    ha, hb = a.half + margin, b.half + margin
    d = b.T[:3, 3] - a.T[:3, 3]
    axes = [Ra[:, i] for i in range(3)] + [Rb[:, i] for i in range(3)]
    axes += [np.cross(Ra[:, i], Rb[:, j]) for i in range(3) for j in range(3)]
    for ax in axes:
        n = np.linalg.norm(ax)
        if n < 1e-9:
            continue
        ax = ax / n
        ra = float(np.sum(ha * np.abs(Ra.T @ ax)))
        rb = float(np.sum(hb * np.abs(Rb.T @ ax)))
        if abs(float(d @ ax)) > ra + rb:
            return False
    return True


def box_block(T_box: np.ndarray, name: str = "box") -> Block:
    """The box body + its tab as one block (box frame: origin bottom center, tab along −y)."""
    T = T_box.copy()
    T[:3, 3] = T_box[:3, :3] @ np.array([0.0, -TAB_OUT / 2, BOX_H / 2]) + T_box[:3, 3]
    return Block(name, T, np.array([HALF, HALF + TAB_OUT / 2, BOX_H / 2]))


PLATE_T = 0.006  # tray plate thickness (m)


def tray_blocks(tray_cfg) -> list[Block]:
    """The calibrated parts tray: its plate (+ the parts sticking out of it) and the space under it down to the
    desk (whatever holds it up). Empty if not calibrated."""
    cal = tray_cfg.calibration() if tray_cfg else None
    if cal is None:
        return []
    o = tray_cfg.layout().outline
    zlo, zhi = -PLATE_T, tray_cfg.protrusion_mm * MM
    T = np.eye(4)
    T[:3, :3] = cal.rot
    T[:3, 3] = cal.to_robot((o["xmin"] + o["xmax"]) / 2, (o["ymin"] + o["ymax"]) / 2, (zlo + zhi) / 2 / MM)
    plate = Block("the parts tray", T, np.array([(o["xmax"] - o["xmin"]) / 2 * MM, (o["ymax"] - o["ymin"]) / 2 * MM,
                                                 (zhi - zlo) / 2]))
    corners = np.array([cal.to_robot(x, y, z / MM) for x in (o["xmin"], o["xmax"]) for y in (o["ymin"], o["ymax"])
                        for z in (zlo, zhi)])
    bottom = float(corners[:, 2].min())
    if bottom < 0.005:
        return [plate]
    ux = cal.rot[:, 0].copy()
    ux[2] = 0.0
    ux = ux / np.linalg.norm(ux) if np.linalg.norm(ux) > 1e-6 else np.array([1.0, 0.0, 0.0])
    uy = np.array([-ux[1], ux[0], 0.0])
    a, b = corners[:, :2] @ ux[:2], corners[:, :2] @ uy[:2]
    Ts = np.eye(4)
    Ts[:3, :3] = np.column_stack([ux, uy, [0.0, 0.0, 1.0]])
    Ts[:3, 3] = ux * (a.min() + a.max()) / 2 + uy * (b.min() + b.max()) / 2 + np.array([0, 0, bottom / 2])
    support = Block("the parts tray (stand)", Ts, np.array([(a.max() - a.min()) / 2, (b.max() - b.min()) / 2, bottom / 2]))
    return [plate, support]


def obstacles(cfg, box_poses: dict[int, np.ndarray], skip: int | None = None) -> list[Block]:
    """3D blocks of the other boxes (at their current spots) and of the calibrated parts tray."""
    return ([box_block(T, f"box {bid}") for bid, T in box_poses.items() if bid != skip]
            + tray_blocks(getattr(cfg, "tray", None)))
