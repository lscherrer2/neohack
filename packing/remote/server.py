"""MCP server exposing slow, high-level arm control to an LLM (Claude, ChatGPT, ...).

Contingency mode: when something goes wrong, a model can look through the cameras and nudge the arm
step by step ("look -> move forward 2 cm -> look -> ..."). It never sees joints; every command is
checked against RemoteLimits before anything moves (see jog.py / safety.py).

Run:
    python -m packing.remote            # real arm, stdio (for Claude Code / Claude Desktop)
    python -m packing.remote --sim      # simulated arm + drawn schematic instead of a camera
    python -m packing.remote --transport streamable-http --port 8765   # HTTP (e.g. ChatGPT connectors)
The arm/cameras are opened on the first tool call (not at startup) and released by `disconnect`.
"""

from __future__ import annotations

import argparse
import threading
import time
from dataclasses import dataclass, fields
from pathlib import Path

import yaml
from mcp.server.mcpserver import Image, MCPServer

from ..config import CellConfig, load_config
from ..hardware.arm import FakeArm, RealArm
from ..motion.controller import Motion
from ..motion.kinematics import gravity_feedforward
from ..poses import PoseStore
from .cameras import CameraSpec, RealCameras, SchematicCamera
from .jog import DIRECTIONS, Jogger, JogResult
from .safety import RemoteLimits

PACKING_DIR = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = PACKING_DIR / "cell.yaml"

INSTRUCTIONS = """\
You are driving a real 6-axis desk robot arm (SO-101) in a slow, conservative contingency mode.

Work in a loop: look -> decide ONE small step -> act -> check the returned image -> repeat.
- Directions are in the robot's own frame, as if standing behind the robot looking where it
  faces: forward/back, left/right, up/down. Camera images may be mirrored or rotated relative to
  that, so after your first move, use the image to work out how directions map onto the picture.
- Default step 2 cm (max 5 cm). Prefer 1-2 cm near objects, people or the table.
- Every command is safety-checked first. "REFUSED, nothing moved" means pick a different
  action; do not retry the same thing in a loop.
- "STOPPED" means the arm hit something or could not keep up: look, then back off (usually
  "up" 1-2 cm) before trying anything else.
- If you are unsure what you see, stop and ask the human instead of guessing.
"""


@dataclass
class RemoteSettings:
    limits: RemoteLimits
    cameras: list[CameraSpec]
    log_file: Path


def load_remote_settings(config_path: Path, cfg: CellConfig) -> RemoteSettings:
    """`remote:` section of cell.yaml (all optional)."""
    raw = (yaml.safe_load(config_path.read_text()) or {}).get("remote") or {}
    known = {f.name for f in fields(RemoteLimits)}
    limits = RemoteLimits(**{k: v for k, v in raw.items() if k in known})
    cameras = [CameraSpec(**c) for c in raw.get("cameras") or []]
    log_file = Path(raw.get("log_file", "data/remote_log.jsonl"))
    if not log_file.is_absolute():
        log_file = config_path.parent / log_file
    return RemoteSettings(limits, cameras, log_file)


class RemoteRig:
    """Arm + motion + jogger + cameras, opened lazily on first use."""

    def __init__(self, config_path: Path, sim: bool):
        self.config_path, self.sim = config_path, sim
        self.cfg = load_config(config_path)
        self.settings = load_remote_settings(config_path, self.cfg)
        if sim:
            self.settings.log_file = self.settings.log_file.with_name("sim_" + self.settings.log_file.name)
        self.lock = threading.Lock()  # one command at a time
        self.jogger: Jogger | None = None
        self.arm = None
        self.cameras = None

    def connect(self) -> Jogger:
        if self.jogger is not None:
            return self.jogger
        cfg = self.cfg
        model = cfg.arm_model()
        if self.sim:
            self.arm = FakeArm(closed=cfg.gripper.closed, opened=cfg.gripper.open)
            self.cameras = SchematicCamera(model, self.settings.limits)
        else:
            self.arm = RealArm(cfg.port, cfg.robot_id, cfg.motion.max_step)
            self.cameras = RealCameras(self.settings.cameras) if self.settings.cameras else None
        self.arm.tune(p_gain=cfg.motion.servo_p, accel=cfg.motion.servo_accel)
        sleep = (lambda s: None) if self.sim else time.sleep
        gravity = None if self.sim else gravity_feedforward(cfg.robot_id, cfg.motion.gravity_gain)
        motion = Motion(self.arm, PoseStore(cfg.poses_file), cfg.motion, cfg.gripper, sleep=sleep, gravity=gravity)
        self.jogger = Jogger(motion, model, self.settings.limits, self.settings.log_file)
        return self.jogger

    def disconnect(self) -> None:
        """Release the serial port and cameras. Motors stay ON, holding the current pose."""
        if self.cameras is not None:
            self.cameras.close()
        if self.arm is not None:
            self.arm.close(release=False)
        self.jogger = self.arm = self.cameras = None

    def images(self) -> list[Image]:
        if self.cameras is None or self.jogger is None:
            return []
        pose = {**self.jogger.motion.cmd}
        return [Image(data=jpg, format="jpeg") for jpg in self.cameras.snapshot(pose).values()]


def build_server(rig: RemoteRig) -> MCPServer:
    server = MCPServer("so101-arm", instructions=INSTRUCTIONS)

    def run(action) -> list:
        with rig.lock:
            try:
                result: JogResult = action(rig.connect())
            except Exception as e:  # noqa: BLE001 — never crash the server; report to the model
                return [f"ERROR (nothing further will move until you call again): {type(e).__name__}: {e}"]
            return [result.text(), *rig.images()]

    @server.tool()
    def look() -> list:
        """Current camera image(s) and tip position. Call this before deciding what to do."""
        return run(lambda j: j.status())

    @server.tool()
    def move(direction: str, cm: float | None = None) -> list:
        """Move the gripper tip in a straight line. direction: forward, back, left, right, up, down
        (robot's own frame). cm: distance, default 2, max 5. Returns the new image(s)."""
        return run(lambda j: j.move(direction, cm))

    @server.tool()
    def tilt(direction: str, degrees: float | None = None) -> list:
        """Tilt the gripper's nose 'up' or 'down' while keeping the tip in place.
        degrees: default 10, max 20."""
        return run(lambda j: j.tilt(direction, degrees))

    @server.tool()
    def twist(direction: str, degrees: float | None = None) -> list:
        """Rotate the wrist 'left' or 'right' about the gripper's axis. degrees: default 10, max 20."""
        return run(lambda j: j.twist(direction, degrees))

    @server.tool()
    def gripper(action: str) -> list:
        """'open' or 'close' the gripper. Closing reports whether something is held."""
        return run(lambda j: j.gripper(action))

    @server.tool()
    def home() -> list:
        """Slowly return the arm to its home pose. Look first: make sure the way is clear."""
        return run(lambda j: j.home())

    @server.tool()
    def disconnect() -> str:
        """Release the arm and cameras for other programs. Motors keep holding the pose."""
        with rig.lock:
            rig.disconnect()
        return "Disconnected. The arm is holding its pose; the next command reconnects."

    return server


def main(argv: list[str] | None = None) -> None:
    p = argparse.ArgumentParser(prog="python -m packing.remote", description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--config", default=str(DEFAULT_CONFIG))
    p.add_argument("--sim", action="store_true", help="simulated arm, schematic image")
    p.add_argument("--transport", choices=["stdio", "streamable-http"], default="stdio")
    p.add_argument("--port", type=int, default=8765, help="HTTP port for streamable-http")
    args = p.parse_args(argv)
    rig = RemoteRig(Path(args.config), args.sim)
    server = build_server(rig)
    try:
        if args.transport == "stdio":
            server.run("stdio")
        else:
            server.run("streamable-http", port=args.port)
    finally:
        rig.disconnect()


__all__ = ["DIRECTIONS", "RemoteRig", "build_server", "main"]
