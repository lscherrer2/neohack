# Packing cell (SO-101, no leader arm)

Teach-and-repeat: you teach each fixed position once, the arm replays them, and an SQLite
inventory records every pick/place so a stopped run can resume. Design notes: `.claude/plans/packing_cell.md`.

Run everything from the repo root with `conda activate lerobot`.

## 1. Describe the table — `packing/cell.yaml`
List your part slots (with SKU), box supply spots, the pack station, output spots, and the
positions inside the box. Each location `L` needs two poses: `L.above` (safe, clear of everything)
and `L.at` (where the gripper closes/opens).

## 2. Teach the poses
```bash
python -m packing check                    # what still needs teaching
python -m packing teach-all --missing      # jog with keys, ENTER saves each, ESC skips
python -m packing teach-all --missing --limp   # or: motors off, move by hand, ENTER saves
python -m packing goto part_A1.at          # verify a pose
```
Keys: q/a pan, w/s shoulder, e/d elbow, r/f wrist_flex, t/g wrist_roll, y/h gripper, +/- step.
Teach `.above` high enough that moving from `home` to it can't hit anything.
First, check which gripper value is open/closed (y/h) and set `gripper.open/closed` in cell.yaml.

## 3. Stock and run
```bash
python -m packing restock      # after physically filling slots / supply spots
python -m packing pack --count 2
python -m packing status       # what's packed / available / where boxes are
python -m packing clear-output # after taking finished boxes away
```
Ctrl+C stops and holds the arm in place. `pack` again resumes from what the inventory says.

## Curved vs straight moves
- **Joint move** (`goto NAME`, plain sequence step): every joint glides together; the tip follows a
  curve (e.g. a 30-unit base turn bulges ~14 mm away from the straight chord).
- **Linear move** (`goto NAME --line`, `{line: NAME}` in the demo): the tip moves in a straight line;
  `motion/kinematics.py` solves IK every 3 mm using the SO-101 URDF (`packing/urdf/`) and placo, and checks
  reach, joint limits and jumps **before** moving.
- `python -m packing where` prints the current tip position (mm; x forward, y left, z up).
- `python -m packing demo` runs `demo_waypoints.yaml`: curves A->B, straight B->A, straight up/down to C.
- `python -m packing demo --file packing/demo_square.yaml`: tip traces a flat 6 cm square, then a 6 cm
  vertical line (the clearest proof the IK works).
- 5-joint arm: tool yaw follows the base, so lines hold the tool's *tilt* (relative to the arm), not its yaw.
- Absolute accuracy depends on how close the calibration mid-pose was to the URDF zero (a few mm);
  straightness and repeatability are much better than that.

## LLM contingency control (`packing/remote/`)
An MCP server that lets Claude / ChatGPT drive the arm slowly with high-level commands, looking at
the camera between steps. Tools: `look`, `move(direction, cm)` (forward/back/left/right/up/down,
robot frame, default 2 cm, max 5), `tilt(up|down, degrees)`, `twist(left|right, degrees)`,
`gripper(open|close)`, `home`, `disconnect`.
- Every command is planned in full and checked against the safe box in `cell.yaml` → `remote:`
  (min/max height, reach, side, joint margin) **before** moving; refused commands move nothing.
- Slow straight lines (20 mm/s) with the usual following-error stop. Every command is logged to
  `data/remote_log.jsonl`.
- Arm and cameras are opened on the first tool call, not at server start; `disconnect` releases them
  (motors keep holding).
- Cameras: list them under `remote.cameras` (find indices with `lerobot-find-cameras opencv`).
```bash
python -m packing.remote --sim          # try it: simulated arm, drawn schematic instead of a camera
claude mcp add so101-arm -e PYTHONPATH=$PWD -- /opt/miniconda3/envs/lerobot/bin/python -m packing.remote
python -m packing.remote --transport streamable-http --port 8765   # HTTP, for other MCP clients
```

## When it stops
- **Missed grasp on a part** → that part is marked `missing`, the next part of the same SKU is tried.
- **Out of stock / no boxes / output full** → box stays put; refill, `restock` / `clear-output`, run `pack` again.
- **Blocked joint (following error)** → arm holds; clear the obstruction.
- **"inventory says the gripper holds …"** → take it out by hand, then e.g.
  `python -m packing set-part 7 available` or `python -m packing set-box BOX-0003 box_in_1`.

## Other
`home`, `release` (motors off), `move-box ID LOC`, `events`, `poses`. Add `--sim` to any command for
a simulated arm (uses separate `data/sim_*` files). Tests: `python -m unittest discover -s packing/tests -t .`

## Files
| file | what |
|---|---|
| `cli.py` | `python -m packing …` (one handler per command) |
| `config.py` | cell.yaml → dataclasses |
| `joints.py` | joint names and value limits |
| `poses.py` | taught poses → `data/poses.json` |
| `hardware/arm.py` | `Arm` protocol, `RealArm` (SO101Follower) / `FakeArm` |
| `hardware/keyboard.py` | jog key map, raw-terminal key reading |
| `motion/controller.py` | `Motion`: smooth moves, settle + following-error checks, sag compensation, grip with stall detection |
| `motion/kinematics.py` | URDF + IK, straight-line paths |
| `cell/inventory.py` | SQLite parts / boxes / event log → `data/inventory.db` |
| `cell/procedures.py` | `Cell`: pick/place, transfer_box, pack_part, resumable `package_box` |
| `tools/teach.py` | teach pendant |
| `tools/demo.py` / `demo_*.yaml` | waypoint demo (curved + straight moves) |
| `tools/tune.py` | servo tuning test (`python -m packing tune`) |
| `service/` | order-taking web service (own README) |
| `cli.py` | `python -m packing …` |
