# Plan: SO-101 packing cell (box transfer + part packing + inventory)

## Goal
Arm moves empty boxes from a supply spot to a packing station, picks parts from fixed,
pre-defined slots into fixed positions inside the box, moves the full box to an output spot,
and keeps a persistent record of what is packed / not packed. Must be resumable after a stop.

## Approach (proven industrial "teach-and-repeat" cell pattern)
- **Joint-space taught poses** (no IK): every location is taught once by jogging or hand-guiding
  the arm. Each location `L` has two poses: `L.above` (safe approach) and `L.at` (grasp/release).
- **Pick/place primitives**: above → open → descend slowly → grip (stall-detect + width check)
  → retreat to above. Place is the mirror.
- **Travel pose**: between stations the arm passes through a safe pose (`home`), so long
  joint-space moves never sweep across the table at grasp height.
- **Inventory = SQLite**, committed *before and after* each physical step. If something is in the
  gripper when a run dies, the next run refuses to start until an operator resolves it.
- **Safety**: speed-limited interpolation, following-error check (stops on collision / stall),
  settle check, library step cap (`max_relative_target`), Ctrl+C holds the arm in place.
- **Sim mode** (`--sim`, FakeArm) so logic can be tested without hardware; unit tests use it.

## Layout
`packing/` (top-level, separate from upstream `src/lerobot`):
- `config.py` – `cell.yaml` → dataclasses
- `joints.py` – joint names / limits
- `poses.py` – named pose store (`data/poses.json`)
- `hardware/arm.py` – `Arm` protocol, `RealArm` (SO101Follower) / `FakeArm`
- `hardware/keyboard.py` – jog key map, raw-terminal input
- `motion/controller.py` – `Motion`: smooth moves, settle, following error, gripper grip/release
- `cell/inventory.py` – SQLite parts/boxes/events
- `cell/procedures.py` – `Cell`: pick/place, transfer_box, pack_part, package_box (resumable job)
- `tools/teach.py` – teach pendant (keyboard jog or limp hand-guiding)
- `cli.py` / `__main__.py` – `python -m packing ...`
- `tests/test_packing.py` – unittest, sim only

## Added: Cartesian motion
- `motion/kinematics.py`: LeRobot `RobotKinematics` (placo) + SO-101 URDF; converts -100..100 units <-> degrees
  with calibration ranges; `line()` = straight tip path, IK every 3 mm, validated before motion.
- `Motion.follow(path, duration)` plays dense paths with smoothstep timing + same safety checks.
- `tools/demo.py` waypoints: joint values, taught poses, or `{from, offset_mm}`; steps `A` (curve) or `{line: A}`.
- Future: use `line` for the last approach of pick/place (`.above` -> `.at`) so the gripper descends vertically.

## Open questions for the user
- Boxes are simple and graspable (user). 15–20 part slots. No external inventory sync for now.
- Still to verify: gripper open/closed direction.
