"""Waypoint demo from a YAML file (default packing/demo_waypoints.yaml).

Sequence entries:
  A            joint move to A   (each joint interpolates; tip follows a gentle CURVE)
  {line: A}    straight-line move of the gripper tip to A (IK at every few mm)
Waypoints:
  {shoulder_pan: -15, ...}           joint values (missing joints = 0 = home)
  {pose: taught_name}                a taught pose
  {from: A, offset_mm: [x, y, z]}    A's tip moved by an offset (x fwd, y left, z up), same orientation
The whole sequence is planned and checked before the arm moves.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
from pathlib import Path
from typing import Callable

import numpy as np
import yaml

from ..joints import BODY
from ..motion.controller import Motion
from ..motion.kinematics import ArmModel
from ..poses import DEFAULTS, PoseStore

Pose = dict[str, float]

MAX_OFFSET_DEFAULT = 20.0  # units from home, if the YAML doesn't say
OFFSET_IK_MAX_ERR_M = 0.002  # an offset_mm waypoint whose IK is further off than this is unreachable


@dataclass
class Demo:
    """A parsed demo file. `steps` are (kind, waypoint name) with kind "joint" or "line"."""

    speed: float  # joint moves, units/s
    line_speed: float  # straight-line moves, average tip speed in mm/s
    dwell: float  # seconds to pause at each checkpoint
    waypoints: dict[str, Pose]
    steps: list[tuple[str, str]]


@dataclass
class PlannedStep:
    """One step with its straight-line path already computed (path/duration/length unset for joint moves)."""

    kind: str
    name: str
    target: Pose
    path: list[Pose] | None = None
    duration: float | None = None  # seconds
    length: float | None = None  # meters


def _resolve_waypoints(raw: dict, poses: PoseStore, model: ArmModel | None) -> dict[str, Pose]:
    """Turn the YAML `waypoints` section into joint poses (plus the built-in `home`)."""
    home = DEFAULTS["home"]
    resolved: dict[str, Pose] = {"home": dict(home)}

    def resolve(name: str, stack: tuple[str, ...] = ()) -> Pose:
        if name in resolved:
            return resolved[name]
        if name not in raw or name in stack:
            raise ValueError(f"waypoint '{name}' undefined (or circular)")
        spec = raw[name] or {}
        if "pose" in spec:
            waypoint = poses.get(spec["pose"])
        elif "offset_mm" in spec:
            if model is None:
                raise ValueError("offset_mm waypoints need kinematics")
            base = resolve(spec["from"], (*stack, name))
            target = model.offset(base, np.asarray(spec["offset_mm"], dtype=float) / 1000)
            waypoint, err = model.ik(base, target)
            if err > OFFSET_IK_MAX_ERR_M:
                raise ValueError(f"waypoint {name} unreachable (IK off by {err * 1000:.1f} mm)")
        else:
            unknown = set(spec) - set(BODY)
            if unknown:
                raise ValueError(f"waypoint {name}: unknown joints {sorted(unknown)} (allowed: {BODY})")
            waypoint = {joint: float(spec.get(joint, home[joint])) for joint in BODY}
        resolved[name] = waypoint
        return waypoint

    for name in raw:
        resolve(name)
    return resolved


def _check_offsets(waypoints: dict[str, Pose], limit: float) -> None:
    home = DEFAULTS["home"]
    for name, waypoint in waypoints.items():
        far = {j: round(waypoint[j], 1) for j in BODY if abs(waypoint[j] - home[j]) > limit}
        if far:
            raise ValueError(f"waypoint {name} exceeds max_offset {limit}: {far}")


def _parse_steps(sequence: list, waypoints: dict[str, Pose]) -> list[tuple[str, str]]:
    steps = []
    for entry in sequence:
        kind, name = ("line", entry["line"]) if isinstance(entry, dict) else ("joint", entry)
        if name not in waypoints:
            raise ValueError(f"sequence uses undefined waypoint '{name}'")
        steps.append((kind, name))
    if steps and steps[0][0] == "line":
        raise ValueError("first sequence step must be a joint move (the arm's start pose is unknown)")
    return steps


def load_demo(path: Path, poses: PoseStore, model: ArmModel | None = None) -> Demo:
    """Parse and validate a demo YAML. Raises ValueError on anything unusable."""
    spec = yaml.safe_load(Path(path).read_text())
    waypoints = _resolve_waypoints(spec.get("waypoints") or {}, poses, model)
    _check_offsets(waypoints, float(spec.get("max_offset", MAX_OFFSET_DEFAULT)))
    return Demo(
        speed=float(spec.get("speed", 10)),
        line_speed=float(spec.get("line_speed_mm_s", 20)),
        dwell=float(spec.get("dwell", 1.0)),
        waypoints=waypoints,
        steps=_parse_steps(spec.get("sequence") or [], waypoints),
    )


def plan(demo: Demo, model: ArmModel | None) -> list[PlannedStep]:
    """Pre-compute every straight line (raises PathError before anything moves)."""
    planned: list[PlannedStep] = []
    previous: Pose | None = None
    for kind, name in demo.steps:
        target = demo.waypoints[name]
        if kind == "line":
            if model is None:
                raise ValueError("line moves need kinematics")
            path = model.line(previous, target)
            length = model.path_length([model.tip(pose) for pose in path])
            duration = length * 1000 / demo.line_speed  # average speed; peak is 1.5x
            planned.append(PlannedStep("line", name, target, path, duration, length))
        else:
            planned.append(PlannedStep("joint", name, target))
        previous = target
    return planned


def run_demo(motion: Motion, demo: Demo, model: ArmModel | None = None, loops: int = 1,
             speed_scale: float = 1.0, log: Callable[[str], None] = print) -> None:
    """Plan the whole sequence, then play it `loops` times, logging a checkpoint after each step."""
    demo = replace(demo, speed=demo.speed * speed_scale, line_speed=demo.line_speed * speed_scale,
                   dwell=demo.dwell / speed_scale)
    steps = plan(demo, model)
    for loop in range(loops):
        prefix = f"[{loop + 1}/{loops}]"
        for index, step in enumerate(steps, 1):
            if step.kind == "line":
                log(f"{prefix} straight line -> {step.name}  "
                    f"({step.length * 1000:.0f} mm, {step.duration:.1f} s)")
                motion.follow(step.path, step.duration, step.name)
            else:
                log(f"{prefix} joint move -> {step.name}")
                motion.move_to(step.target, speed=demo.speed)
            log(_checkpoint_message(motion.read(), step, index, len(steps), model))
            motion.sleep(demo.dwell)


def _checkpoint_message(actual: Pose, step: PlannedStep, index: int, total: int,
                        model: ArmModel | None) -> str:
    message = f"    checkpoint {index}/{total} '{step.name}' reached"
    if model is not None:
        tip = model.tip(actual)
        error_mm = np.linalg.norm(tip - model.tip(step.target)) * 1000
        message += f" — tip {np.round(tip * 1000).astype(int)} mm, {error_mm:.1f} mm from target"
    return message
