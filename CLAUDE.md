# CLAUDE.md

Drives a **single SO-101 follower arm (Hiwonder kit, no leader arm)** straight over the Feetech servo SDK —
**no LeRobot** (removed Nov 2026; our driver keeps LeRobot's units and calibration format). The project is
`packing/`: a teach-and-repeat cell that moves boxes, packs parts from fixed slots, and tracks inventory.

## Environment
- `.venv/` (Python 3.13): `python3 -m venv .venv && .venv/bin/pip install -r requirements.txt`. Deps: `feetech-servo-sdk`
  (`scservo_sdk`), `pyserial`, `numpy`, `pyyaml`, `placo` (IK), `mcp` 2.x (`MCPServer`, `from mcp import Client`),
  `opencv-python`, `fastapi`/`uvicorn`, `anthropic` 2.x. Run things as `.venv/bin/python -m packing ...`.
- Arm port: `/dev/cu.usbmodem5CE51213501` (macOS). Check with `ls /dev/cu.*`.
- Only one process can own the port. `lsof /dev/cu.usbmodem5CE51213501` before running anything, and
  never kill a user's session (e.g. `keys`) — ask them to quit it.
- Tests (sim only, no hardware): `.venv/bin/python -m unittest discover -s packing/tests -t .` from repo root.
  No pytest in the env.

## Hardware facts
- Assembled Hiwonder kit: STS3215 servos, IDs 1–6 already set (`shoulder_pan`…`gripper`), 1 Mbps. Never change
  servo IDs/baud.
- Calibrated as robot id `my_awesome_follower_arm`: values live in the servo EEPROM **and**
  `packing/calibration/my_awesome_follower_arm.json` (LeRobot JSON format; `RealArm` refuses to connect if they
  differ). Tool: `python -m packing.tools.calibrate scan|backup|center|range|restore` (torque off throughout).
  Don't recalibrate casually: it changes what every taught pose / dataset value means.
  **Recalibrated 2026-11-07** (old values in `packing/calibration/*.backup_20261107_114046.json`): poses taught
  before that are ~7° off on lift/elbow — re-teach.
- Units: body joints -100..100 (0 = calibration mid pose = `home`), gripper 0..100.
  Shoulder/elbow sag ~5 units under gravity when holding a pose.
- Tracking (measured with `python -m packing tune`, Nov 2026): connect sets servo P=16, accel=254 (LeRobot's values).
  Main error is **gravity sag on shoulder_lift** (~8 units at rest = ~5 cm at the tip); jitter is
  small (~0.6 mm). P=32 halves sag but the user found it too stiff, so `servo_p` default is 24 (middle);
  accel=20 kept (RAM only, re-applied each connect via `RealArm.tune`) + software sag compensation (`motion.sag_gain`, outer integral loop in `Motion._tick`).
- Verified directions (Nov 2026): gripper 0 = closed, 100 = open; `wrist_flex` + = wrist tips up.

## Moving the arm — rules
- Physical motion is outward-facing: get the user's explicit OK ("go") before every run that moves
  the arm; say what will move and how far. Read-only connects/reads are fine.
- Torque off = arm drops. Warn the user to hold it first. Use `--keep-torque` / `close(release=False)`
  when you can't prompt.
- Connecting (`SO101`) never drops torque: settings go to RAM with EEPROM locked, then torque on at goal = present.
- Holding a pose heats the servos (gripper hit 60 °C after ~30 min; limit ~70). Release when idle.
- Right after writing `Homing_Offset`, `Present_Position` reads can be stale for a moment.

## Layout
- `packing/` — the cell (see `packing/README.md`, design in `.claude/plans/packing_cell.md`):
  - `joints.py` joint names/limits · `config.py` cell.yaml → dataclasses · `poses.py` taught-pose store · `cli.py`
  - `hardware/` `so101.py` Feetech driver (`FeetechBus` registers/sync r/w, `SO101` units + calibration) ·
    `arm.py` Arm protocol, RealArm/FakeArm · `keyboard.py` jog key map + raw-terminal input
  - `motion/` `controller.py` `Motion`: smooth joint moves, `follow()` for paths, settle + following-error
    checks, sag compensation, stall-detect grip · `kinematics.py` URDF + placo IK (`PlacoKinematics`), straight-line paths
  - `cell/` `inventory.py` SQLite · `procedures.py` `Cell`: pick/place + resumable `package_box`
  - `tools/` `teach.py` teach pendant · `demo.py` waypoint demo · `tune.py` servo tuning test · `calibrate.py`
  - `service/` first, Vapi-first order service (webhook + callbacks; `packing/service/README.md`,
    `VAPI_SETUP.md`). **Parked** — kept as reference; `shop/` reuses its `orders.py` (OrderStore),
    `vapi.py` (Catalog, parse_items) and `worker.py` (RobotPacker).
  - `shop/` **current** order stack (see "Shop" below and `packing/shop/README.md`)
  - Config `packing/cell.yaml`; data in `packing/data/` (`poses.json`, `inventory.db`; `--sim` uses `sim_*`).
  - `packing/urdf/so101_new_calib.urdf` — from TheRobotStudio/SO-ARM100, meshes stripped (kinematics only).
  - `packing/calibration/` — per-arm calibration JSON (committed).
- `packing/remote/` — LLM contingency control over MCP (`python -m packing.remote [--sim]`, mcp 2.x:
  `from mcp.server.mcpserver import MCPServer`). `jog.py` direction commands → checked straight lines;
  `safety.py` RemoteLimits safe box; `cameras.py` cv2.VideoCapture or sim schematic; `server.py` tools + LLM instructions.
- `camera/` — wrist camera access (`camera/README.md`). Claude's shell has no macOS camera permission:
  the **user** runs `python camera/cam_server.py`, then fetch frames from `http://127.0.0.1:8765/snap.jpg`
  (check `/info` first). Opens only the "icspring camera" by name — never other cameras.
- Parts tray (planned, `.claude/plans/tray_layout.md`): `part.dxf` = tray layout (33 pockets: AA, 9V,
  piezo, glue stick, breadboard), `box.step` = packing box with grab tab (units m). Not implemented yet.
- CLI: `python -m packing <check|teach|teach-all|goto [--line]|where|home|release|demo|tune|restock|status|events|set-part|set-box|clear-output|pack|move-box> [--sim]`

## Shop (current ordering stack) — `packing/shop/`, plan `.claude/plans/shop_slim.md`
- `python -m packing.shop start [--robot] [--auto]` = one process: FastAPI server on :8000 + MCP tools
  at `/mcp` + dashboard at `/dashboard` + robot agent thread (simulated packing unless `--robot`).
  `serve` / `robot` / `chat` run the pieces separately. Only one server can hold :8000 — check
  `lsof -iTCP:8000` and ask the user to stop theirs; never kill it. Test on a spare port.
- Flow: ordering agent (Claude Haiku `claude-haiku-5-5`, key from `.env.local` — git-ignored, never
  print it) uses only the MCP tools `list_products / place_order / check_order / cancel_order`
  (`tools.py`, replies are short spoken sentences). Web chat (`POST /api/chat`) and terminal chat share
  `chat.Conversation`.
- **Manual dispatch is the default**: orders arrive `queued` (shown as "received"); the dashboard's
  **Action** → `ready`; the robot agent only claims `ready` orders (`--auto` claims `queued`).
  **Delete** allowed for queued/ready/failed/cancelled. Statuses: queued → ready → packing → packed | failed.
- **Real data only** (user asked: no placeholder stock): orders in `packing/data/shop_orders.db`;
  stock from the cell inventory `packing/data/inventory.db` via `shelf.Shelf` (opens the DB per call —
  SQLite connections are per-thread). Inventory starts empty: dashboard **Restock** after filling slots.
  `ShelfSimPacker` consumes real parts so stock stays true. Catalog names: `catalog:` in `cell.yaml`
  (still placeholder widget/gadget). Tests/live checks must use a temp copy of `cell.yaml` so they don't
  write the real databases.
- Robot agent is the only thing that opens the arm; it polls `POST /robot/claim` (returns the in-progress
  order first → resume), reports `done`/`failed`, and **exits on a packing failure** (human checks arm).
- Dashboard (`static/dashboard.html`, single file, no deps): status plate (yellow packing / steel idle /
  red offline-or-failed), chat, shelf, orders with Action/Delete, live activity via SSE
  (`/api/events/stream`, in-memory `events.EventLog`, last 500, lost on restart). Avoid CSS class names
  that equal event kinds (`tool/robot/order/chat/error/system`) — they collide with `.events li.<kind>`.
- Vapi later: put it in front of the same MCP server (Vapi `mcp` tool type) or the old webhook adapter.

## Conventions
- Every location `L` = poses `L.above` + `L.at`; between stations go via `travel_pose` (`home`).
- Inventory is committed before and after each physical step; `in_gripper` items block runs until resolved.
- SO-101 has 5 body DOF: IK targets must not fix tool yaw (it's tied to `shoulder_pan`). `ArmModel`
  splits R = Rz(azimuth) @ local and only holds/interpolates the local part — keep it that way.
- Joint moves = curved tip path; `{line: X}` / `goto --line` = straight tip path via IK, fully planned
  and validated (reach, joint limits, jumps) before moving.
- Keep new features testable with `FakeArm` + `--sim`; add tests in `packing/tests/`.
