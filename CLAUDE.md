# CLAUDE.md

Drives a **single SO-101 follower arm (Hiwonder kit, no leader arm)** straight over the Feetech servo SDK —
**no LeRobot** (removed Nov 2026; our driver keeps LeRobot's units and calibration format). The project is
`packing/`: a teach-and-repeat cell that moves boxes, packs parts from a calibrated parts tray (DXF layout), and
tracks inventory; `packing/shop/` puts an ordering agent + React dashboard (orders, live 3D arm, tray calibration) on top.

## Environment
- `.venv/` (Python 3.13): `python3 -m venv .venv && .venv/bin/pip install -r requirements.txt`. Deps: `feetech-servo-sdk`
  (`scservo_sdk`), `pyserial`, `numpy`, `pyyaml`, `placo` (IK), `mcp` 2.x (`MCPServer`, `from mcp import Client`),
  `opencv-python`, `fastapi`/`uvicorn`, `anthropic` 2.x, `ezdxf` (tray DXF), `scipy` (rigid fit).
  Run things as `.venv/bin/python -m packing ...`.
- Frontend (`packing/shop/web/`): Node + npm (Vite, React 19, TS, three / @react-three/fiber / drei).
  `npm install && npm run build` → `packing/shop/static/app/` (**built output is committed** so Python-only
  runs work — rebuild and commit it after UI changes). `npm run dev` proxies to `VITE_API_TARGET` (default :8000).
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
- **Joint angles for kinematics** (`ArmModel`, fixed 2026-11-07): URDF 0° = the pose held during `calibrate center`
  (raw `HALF_TURN` = 2047), deg = (raw − 2047)·360/4095. NOT the middle of the recorded range (unit 0): the SO-101's
  limits are asymmetric (lift 5°, elbow 8.7°, wrist 4° apart here), which put the tip ~2–4 cm low. With the fix, the
  jaw resting on the table reads z ≈ +5 mm. Remaining error = how well the (eyeballed) center pose was held; a
  table-touch joint-offset calibration is the planned fix if needed. Units / taught poses are unaffected.

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
    checks, sag compensation, stall-detect grip · `kinematics.py` URDF + placo IK (`PlacoKinematics`), straight-line
    paths, `ArmModel.link_frames()` (FK of every drawn link for the 3D view)
  - `cell/` `inventory.py` SQLite · `procedures.py` `Cell`: pick/place + resumable `package_box`
  - `tools/` `teach.py` teach pendant · `demo.py` waypoint demo · `tune.py` servo tuning test · `calibrate.py`
    (servo calibration) · `tray_calibrate.py` hand-guided tray calibration (CLI)
  - `tray/` parts tray: `dxf.py` ezdxf reader (circles + closed rect loops) · `layout.py` pockets by type ·
    `calibration.py` tray→robot rigid fit + checks · `session.py` `CalibrationSession` (dashboard calibration)
  - `hardware/telemetry.py` `TelemetryArm` (publishes joints on read, throttled; `http_publisher` non-blocking)
  - `service/` first, Vapi-first order service (webhook + callbacks; `packing/service/README.md`,
    `VAPI_SETUP.md`). **Parked** — kept as reference; `shop/` reuses its `orders.py` (OrderStore),
    `vapi.py` (Catalog, parse_items) and `worker.py` (RobotPacker).
  - `shop/` **current** order stack (see "Shop" below and `packing/shop/README.md`)
  - Config `packing/cell.yaml`; data in `packing/data/` (`poses.json`, `inventory.db`, `tray_cal.json`;
    `--sim` uses `sim_*`).
  - `packing/urdf/so101_new_calib.urdf` — from TheRobotStudio/SO-ARM100, meshes stripped (kinematics only).
  - `packing/calibration/` — per-arm calibration JSON (committed).
- `packing/remote/` — LLM contingency control over MCP (`python -m packing.remote [--sim]`, mcp 2.x:
  `from mcp.server.mcpserver import MCPServer`). `jog.py` direction commands → checked straight lines;
  `safety.py` RemoteLimits safe box; `cameras.py` cv2.VideoCapture or sim schematic; `server.py` tools + LLM instructions.
- `camera/` — wrist camera access (`camera/README.md`). Claude's shell has no macOS camera permission:
  the **user** runs `python camera/cam_server.py`, then fetch frames from `http://127.0.0.1:8765/snap.jpg`
  (check `/info` first). Opens only the "icspring camera" by name — never other cameras.
- `part.dxf` = parts tray layout (mm); `box.step` = packing box with grab tab (STEP, units m; box handling is TODO).
- CLI: `python -m packing [--sim] <check|teach|teach-all|goto [--line]|where|home|release|demo|tune|restock|tray|tray-calibrate|status|events|set-part|set-box|clear-output|pack|move-box>`
  (`--sim` goes **before** the command).

## Parts tray — plan `.claude/plans/tray_layout.md`
- `tray:` in `cell.yaml`: DXF path, item types (feature circle/rect/pin_pair + size mm, `grip_axis`, name),
  `reference_points`, protrusion 10 mm, extract ≥120 mm (glue stick length). 33 pockets: `aa_1..7`, `9v_1..5`,
  `piezo_1..8` (pick point = midpoint of the pin pair), `glue_1..8`, `breadboard_1..5`; `_1` = row nearest the arm.
- Tray frame = DXF x/y, z out of the face; DXF x runs left→right as seen from the arm. Grip: fingers close along
  tray x for all but the breadboard (tray y). Catalog names come from the tray types; `part_slots` is empty;
  one item per order (`box_slots` has one slot). `cfg.stock_slots` = part_slots + tray pockets.
- Calibration: motors off, touch the closed gripper tip (`gripper_frame_link`, red dot in the 3D view) to reference
  points → scipy rigid fit, refused above 3 mm RMS. `tray.calibration_mode`: **corners** (default; the middle of each
  rounded plate corner on the top edge, ids `corner_bl/tl/br/tr`, `TrayLayout.corners()`) or **pockets**
  (`reference_points`: `aa_1`, `aa_7`, `breadboard_1`, `breadboard_5`). Any pocket/corner can be added as an extra
  point. CLI `tray-calibrate` or the dashboard Calibration tab (Corners/Pockets switch). Sim saves `sim_tray_cal.json`.
- **Go-to preview** (`tray/goto.py` plan, `shop/goto_runner.py` run; dashboard "Go to point" box, `POST /api/arm/goto`):
  tip to `standoff_mm` (default 12.7 = ½ in) out along the tray normal from a corner/pocket, tool axis
  (gripper_frame +z) = inward normal projected into the arm's plane (5-DOF; leftover tilt reported). Free move to
  60 mm out, then a straight line in. Always dry-run first (`confirm_move: false`) → UI confirm dialog → move;
  `POST /api/arm/stop` halts. Uses the live fit while calibrating if good, else the saved calibration. Turns the
  motors on (holding) if they were off; `POST /api/arm/release` turns them off again for more hand-guided capture.
- 5-DOF limit (accepted by the user): off-center pockets are gripped tilted (no wrist yaw); `tray` / the fit
  report shows the worst angle per item. Picking from pockets (`Cell`) is **not implemented yet**; nor box/drop spot.

## Shop (current ordering stack) — `packing/shop/`, plan `.claude/plans/shop_slim.md`
- `python -m packing.shop start [--robot] [--auto]` = one process: FastAPI server on :8000 + MCP tools
  at `/mcp` + dashboard at `/dashboard/` + robot agent thread (simulated packing unless `--robot`).
  `serve [--robot]` / `robot` / `chat` run the pieces separately. Only one server can hold :8000 — check
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
  `ShelfSimPacker` consumes real parts so stock stays true. Tray pockets are marked full/empty from the dashboard
  (humans restock; trust that state). Tests/live checks must use a temp copy of `cell.yaml`
  (`packing.tests.test_tray.temp_config`) so they don't write the real databases / calibration.
- **Arm ownership**: `shop/arm_hub.py` `ArmHub` owns the arm in the server process (real with `--robot`, FakeArm
  otherwise; connects on demand, polls ~10 Hz, FK frames). In-process packing borrows it (`RobotPacker(arm_provider=
  hub.borrow)`); a separate `robot --robot` process opens its own arm and posts `/robot/telemetry`. Calibration
  turns torque off only after a `confirm_hold` from the UI; `/robot/claim` hands out nothing while calibrating.
- Robot agent polls `POST /robot/claim` (returns the in-progress order first → resume), reports
  `done`/`failed`, and **exits on a packing failure** (human checks arm).
- Dashboard = React app (`packing/shop/web/`, served from `static/app/` at `/dashboard/`; old
  `static/dashboard.html` is only a fallback if the build is missing). Tabs: **Orders** (status plate, chat,
  stock + Restock, 2D tray map — click a pocket to toggle full/empty, orders with Action/Delete, live activity via
  SSE `/api/events/stream`, in-memory `events.EventLog`, last 500) and **Calibration** (3D arm + tray view from
  `/api/arm/stream`, "Preview pose" toggle = generic poses without hardware via `/api/arm/preview`, capture
  buttons per reference corner + any pocket, live fit, Save/Finish). API contract: `.claude/plans/arm_viz.md`.
  3D arm = stick figure (line per segment, arc per joint bend, jaw line) + bright red dot at the tool point. Joints are
  projected into the arm's vertical plane (URDF lift/elbow/wrist origins sit 18.3 mm along their axes — fake jogs
  otherwise); the tool point is the true FK position. Avoid CSS class names that equal event kinds
  (`tool/robot/order/chat/error/system`) — they collide with the activity list styles.
- Vapi later: put it in front of the same MCP server (Vapi `mcp` tool type) or the old webhook adapter.

## Conventions
- Every location `L` = poses `L.above` + `L.at`; between stations go via `travel_pose` (`home`).
- Inventory is committed before and after each physical step; `in_gripper` items block runs until resolved.
- SO-101 has 5 body DOF: IK targets must not fix tool yaw (it's tied to `shoulder_pan`). `ArmModel`
  splits R = Rz(azimuth) @ local and only holds/interpolates the local part — keep it that way.
- Joint moves = curved tip path; `{line: X}` / `goto --line` = straight tip path via IK, fully planned
  and validated (reach, joint limits, jumps) before moving.
- Keep new features testable with `FakeArm` + `--sim`; add tests in `packing/tests/`.
- **Keep this file current**: whenever a change affects layout, commands, APIs, config, data files or rules here,
  update CLAUDE.md (and the plans it points to) in the same change, without being asked.
- **Prefer a well-fitting library over custom code** for anything complex (math, geometry, file formats,
  UI). Write it ourselves only if no good library fits, or the library has a real downside (huge, unmaintained,
  wrong license, poor fit). Add new deps to `requirements.txt`. E.g. DXF via `ezdxf`, rigid fits via `scipy`.
