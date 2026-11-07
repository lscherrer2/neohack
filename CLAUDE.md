# CLAUDE.md

Fork of Hugging Face LeRobot (Hiwonder's SO-ARM101 distribution, with local tweaks in `src/lerobot`)
driving a **single SO-101 follower arm with no leader arm**. On top of it sits `packing/`: a
teach-and-repeat cell that moves boxes, packs parts from fixed slots, and tracks inventory.

## Environment
- `conda activate lerobot` (Python 3.10, `/opt/miniconda3/envs/lerobot`). Installed: `.[feetech]`, `placo` (IK), `mcp` 2.x.
- Arm port: `/dev/cu.usbmodem5CE51213501` (macOS). Check with `ls /dev/cu.*`.
- Only one process can own the port. `lsof /dev/cu.usbmodem5CE51213501` before running anything, and
  never kill a user's session (e.g. `keys`) — ask them to quit it.
- Tests (sim only, no hardware): `python -m unittest discover -s packing/tests -t .` from repo root.
  No pytest in the env.

## Hardware facts
- Assembled Hiwonder kit: servo IDs 1–6 already set (`shoulder_pan`…`gripper`). Never run
  `lerobot-setup-motors` — that's for DIY kits only.
- Calibrated as robot id `my_awesome_follower_arm`; file at
  `~/.cache/huggingface/lerobot/calibration/robots/so101_follower/my_awesome_follower_arm.json`.
  Don't recalibrate casually: it changes what every taught pose / dataset value means.
  `wrist_roll` was re-centered by hand (homing offset -73) because its mid pose was off.
- Units: body joints -100..100 (0 = calibration mid pose = `home`), gripper 0..100.
  Shoulder/elbow sag ~5 units under gravity when holding a pose.
- Tracking (measured with `python -m packing tune`, Nov 2026): LeRobot sets servo P=16, accel=254.
  Main error is **gravity sag on shoulder_lift** (~8 units at rest = ~5 cm at the tip); jitter is
  small (~0.6 mm). P=32 halves sag but the user found it too stiff, so `servo_p` default is 24 (middle);
  accel=20 kept (RAM only, re-applied each connect) + software sag compensation (`motion.sag_gain`, outer integral loop in `Motion._tick`).
- Which gripper value is open vs closed is **not yet verified** (`gripper.open/closed` in `packing/cell.yaml`).

## Moving the arm — rules
- Physical motion is outward-facing: get the user's explicit OK ("go") before every run that moves
  the arm; say what will move and how far. Read-only connects/reads are fine.
- Torque off = arm drops. Warn the user to hold it first. Use `--keep-torque` / `close(release=False)`
  when you can't prompt.
- `max_relative_target` must be a **float** (`ensure_safe_goal_position` raises `TypeError` on int).
- Right after writing `Homing_Offset`, `Present_Position` reads can be stale for a moment.

## Layout
- `examples/so101_no_leader.py` — quick standalone `demo` / `keys` / `home` control.
- `packing/` — the cell (see `packing/README.md`, design in `.claude/plans/packing_cell.md`):
  - `joints.py` joint names/limits · `config.py` cell.yaml → dataclasses · `poses.py` taught-pose store · `cli.py`
  - `hardware/` `arm.py` Arm protocol, RealArm/FakeArm · `keyboard.py` jog key map + raw-terminal input
  - `motion/` `controller.py` `Motion`: smooth joint moves, `follow()` for paths, settle + following-error
    checks, sag compensation, stall-detect grip · `kinematics.py` URDF + placo IK, straight-line paths
  - `cell/` `inventory.py` SQLite · `procedures.py` `Cell`: pick/place + resumable `package_box`
  - `tools/` `teach.py` teach pendant · `demo.py` waypoint demo · `tune.py` servo tuning test
  - `service/` order-taking web service (see `packing/service/README.md`)
  - Config `packing/cell.yaml`; data in `packing/data/` (`poses.json`, `inventory.db`; `--sim` uses `sim_*`).
  - `packing/urdf/` — SO-101 URDF + meshes from TheRobotStudio/SO-ARM100 (`so101_new_calib.urdf`).
- `packing/remote/` — LLM contingency control over MCP (`python -m packing.remote [--sim]`, mcp 2.x:
  `from mcp.server.mcpserver import MCPServer`). `jog.py` direction commands → checked straight lines;
  `safety.py` RemoteLimits safe box; `cameras.py` OpenCV or sim schematic; `server.py` tools + LLM instructions.
- CLI: `python -m packing <check|teach|teach-all|goto [--line]|where|home|release|demo|tune|restock|status|events|set-part|set-box|clear-output|pack|move-box> [--sim]`

## Conventions
- Every location `L` = poses `L.above` + `L.at`; between stations go via `travel_pose` (`home`).
- Inventory is committed before and after each physical step; `in_gripper` items block runs until resolved.
- SO-101 has 5 body DOF: IK targets must not fix tool yaw (it's tied to `shoulder_pan`). `ArmModel`
  splits R = Rz(azimuth) @ local and only holds/interpolates the local part — keep it that way.
- Joint moves = curved tip path; `{line: X}` / `goto --line` = straight tip path via IK, fully planned
  and validated (reach, joint limits, jumps) before moving.
- Keep new features testable with `FakeArm` + `--sim`; add tests in `packing/tests/`.
