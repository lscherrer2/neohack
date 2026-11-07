# Camera access (wrist camera on the SO-101 gripper)

Rescued from the old `camera-docs` worktree notes (2026-11-06) + the camera server script, which
previously lived only at `~/lerobot-tools/cam_server.py` (outside the repo).

## Why a server
Claude Code's shell has **no macOS camera permission**, so OpenCV / camera probing fails there
with `not authorized to capture video`. Get images through this local camera server instead.

- Server: `camera/cam_server.py`. The **user** runs it in their own terminal (one that has camera permission):
  `conda activate lerobot && python camera/cam_server.py`
- It opens **only** the USB robot camera, by name (`"icspring camera"`, wrist/hand-eye camera on the
  gripper), via ffmpeg AVFoundation. **Never open other cameras** (built-in FaceTime, the user's iPhone via
  Continuity) — the user asked that they are not touched at all. Don't probe OpenCV indices.
- Bound to `127.0.0.1:8765`. Endpoints:
  - `curl -s -o /tmp/x.jpg http://127.0.0.1:8765/snap.jpg` → latest frame (640x480 JPEG); then view it with Read
  - `/info` → JSON (name, size, fps, frames_received, alive, error) — check this first; if the connection is
    refused, ask the user to start the server
  - `/sharpness` → JSON focus score (Laplacian variance, center 50%) + brightness
  - `/` → live view + focus meter (for the user's browser); `/stream` → MJPEG
- Options: `--size WxH --fps N --name "<camera name>" --port P`, `--test-pattern` (synthetic image, for
  testing the server without a camera — run it on a different port so the user's server isn't disturbed).

## Known state (2026-11-06)
- Focused (sharpness ~500 on the gripper view; ~8 when blurry). Warm orange cast and some glare on the
  top/right edges.
- The OpenCV index for this camera was `1` at one point, but indices shift (and don't match macOS name
  order), so don't rely on them. If cameras get wired into robot code (e.g. `packing/remote/cameras.py`),
  select by name instead.
