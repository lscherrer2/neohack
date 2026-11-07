"""Camera snapshots for the LLM: real OpenCV cameras, or a drawn schematic in sim mode."""

from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np

from ..motion.kinematics import ArmModel
from .safety import RemoteLimits

MAX_WIDTH = 640  # downscale big frames: plenty for an LLM, keeps responses small
JPEG_QUALITY = 80


@dataclass
class CameraSpec:
    name: str
    index_or_path: int | str
    width: int = 640
    height: int = 480
    fps: int = 30


def _jpeg(image_bgr: np.ndarray) -> bytes:
    h, w = image_bgr.shape[:2]
    if w > MAX_WIDTH:
        image_bgr = cv2.resize(image_bgr, (MAX_WIDTH, int(h * MAX_WIDTH / w)))
    ok, buf = cv2.imencode(".jpg", image_bgr, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
    if not ok:
        raise RuntimeError("JPEG encoding failed")
    return buf.tobytes()


class RealCameras:
    """OpenCV cameras, opened on first use."""

    WARMUP_FRAMES = 5  # first frames after opening are often dark / stale

    def __init__(self, specs: list[CameraSpec]):
        self.specs = specs
        self._cams: dict[str, cv2.VideoCapture] = {}

    def _open(self) -> None:
        for spec in self.specs:
            if spec.name in self._cams:
                continue
            cam = cv2.VideoCapture(spec.index_or_path)
            if not cam.isOpened():
                raise RuntimeError(f"camera '{spec.name}' ({spec.index_or_path}) did not open")
            cam.set(cv2.CAP_PROP_FRAME_WIDTH, spec.width)
            cam.set(cv2.CAP_PROP_FRAME_HEIGHT, spec.height)
            cam.set(cv2.CAP_PROP_FPS, spec.fps)
            for _ in range(self.WARMUP_FRAMES):
                cam.read()
            self._cams[spec.name] = cam

    def snapshot(self, pose: dict[str, float] | None = None) -> dict[str, bytes]:
        self._open()
        out = {}
        for name, cam in self._cams.items():
            ok, bgr = cam.read()
            if not ok:
                raise RuntimeError(f"camera '{name}' returned no frame")
            out[name] = _jpeg(bgr)
        return out

    def close(self) -> None:
        for cam in self._cams.values():
            cam.release()
        self._cams.clear()


class SchematicCamera:
    """Sim stand-in: draws the arm (side view and top view) with the safe zone, from FK."""

    LINKS = ["base_link", "shoulder_link", "upper_arm_link", "lower_arm_link", "wrist_link",
             "gripper_link", "gripper_frame_link"]
    PX_PER_M = 600

    def __init__(self, model: ArmModel, limits: RemoteLimits):
        self.model, self.limits = model, limits

    def _points(self, pose: dict[str, float]) -> np.ndarray:
        self.model.fk(pose)
        robot = self.model.kin.robot
        return np.array([np.asarray(robot.get_T_world_frame(f))[:3, 3] for f in self.LINKS])

    def snapshot(self, pose: dict[str, float] | None = None) -> dict[str, bytes]:
        pts = self._points(pose)
        img = np.full((320, 640, 3), 255, np.uint8)
        s, lim = self.PX_PER_M, self.limits
        # side view (x right, z up), left half
        side = lambda x, z: (int(20 + x * s * 0.6), int(300 - z * s * 0.6))  # noqa: E731
        cv2.line(img, side(-0.02, 0), side(0.48, 0), (120, 120, 120), 2)  # table
        cv2.rectangle(img, side(lim.min_reach_m, lim.max_z_m), side(lim.max_reach_m, lim.min_z_m), (0, 170, 0), 1)
        for a, b in zip(pts, pts[1:]):
            cv2.line(img, side(a[0], a[2]), side(b[0], b[2]), (200, 80, 0), 3)
        cv2.circle(img, side(pts[-1][0], pts[-1][2]), 5, (0, 0, 255), -1)
        cv2.putText(img, "SIDE (x fwd ->, z up)", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
        # top view (x up the image, y left), right half
        top = lambda x, y: (int(480 - y * s * 0.6), int(300 - x * s * 0.6))  # noqa: E731
        cv2.circle(img, top(0, 0), int(lim.max_reach_m * s * 0.6), (0, 170, 0), 1)
        for a, b in zip(pts, pts[1:]):
            cv2.line(img, top(a[0], a[1]), top(b[0], b[1]), (200, 80, 0), 3)
        cv2.circle(img, top(pts[-1][0], pts[-1][1]), 5, (0, 0, 255), -1)
        cv2.putText(img, "TOP (fwd ^, left <-)", (330, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
        tip = pts[-1] * 100
        cv2.putText(img, f"tip x={tip[0]:.1f} y={tip[1]:.1f} z={tip[2]:.1f} cm", (10, 315),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 1)
        return {"schematic": _jpeg(img)}

    def close(self) -> None:
        pass
