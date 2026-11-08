# CLAUDE.md

**Start here for what to build next: `.claude/plans/ROADMAP.md`** (the stack, layer by layer, and the order of work).

Drives a **single SO-101 follower arm (Hiwonder kit, no leader arm)** straight over the Feetech servo SDK —
**no LeRobot** (removed Nov 2026; our driver keeps LeRobot's units and calibration format). The project is
`packing/`: a teach-and-repeat cell that moves boxes, packs parts from a calibrated parts tray (DXF layout), and
tracks inventory; `packing/shop/` puts an ordering agent + React dashboard (orders, live 3D arm, tray calibration) on top.

## Environment
- `.venv/` (Python 3.13): `python3 -m venv .venv && .venv/bin/pip install -r requirements.txt`. Deps: `feetech-servo-sdk`
  (`scservo_sdk`), `pyserial`, `numpy`, `pyyaml`, `placo` (IK), `mcp` 2.x (`MCPServer`, `from mcp import Client`),
  `opencv-python`, `fastapi`/`uvicorn`, `anthropic` 2.x, `ezdxf` (tray DXF), `scipy` (rigid fit), `fastmcp` 4.x
  (remote-agent MCP at `/agent/mcp`; coexists with `mcp` 2.x).
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
- Tracking (Nov 2026, measured by tracing `Motion` ticks): servos are P-only (P=31 via `servo_p`; was 24 — 31 tracked best of 17/24/31) and trail a
  moving command by ~0.1 s, and gravity makes joints sag ~1-2 units (more mid-move: static friction hides part
  of it at rest). Motion stack, all in `cell.yaml` `motion:`: `servo_accel: 254` (20 doubled lag — the servo's
  own profile fights our streamed trajectory) · `lead_s: 0.077` velocity feed-forward (0.1 s measured at P=24, scaled) · `gravity_gain: 3.21`
  units/N·m model gravity feed-forward (fit at P=24: shoulder 4.17, elbow 4.12, residual ≤0.2 units; scaled by
  24/31 for P=31 — sag and lag go ~1/P; re-fit if P changes; real arm only) ·
  sag integral `bias` learns **only at rest** (learning lag mid-move caused overshoot / flailing) ·
  `settle_tol: 1`. Result: joints in step within ~3 %, lag ~1 unit, 50 mm IK line within ~5 mm.
- Directions (Nov 2026): gripper 0 = closed, 100 = open; `elbow_flex` + = forearm down; `wrist_flex` + = gripper
  tips **down** (an early note said "up" — wrong; the model + a 15° pitch observation settle it).
- **Kinematics zero** (fixed 2026-11-07, independently on both branches): URDF 0° = the calibration pose = raw
  2047 (`HALF_TURN`), deg = (raw − 2047)·360/4095, NOT unit 0 (= range middle). The limits are asymmetric (lift 5°,
  elbow 8.7°, wrist 4° here); the range middle (as LeRobot does) left the gripper ~15° off level and the tip 2–4 cm
  low. `ArmModel.zero_deg` handles it (drive-mode sign included). With the fix the jaw resting on the table reads
  z ≈ +5…+10 mm. Remaining error = how well the eyeballed calibration pose was held; a table-touch joint-offset fit
  is the planned fix if needed. Units / taught poses are unaffected.
- Workspace: there is a **monitor near the arm** — it was hit during a wide pose sweep (2026-11-07). The
  height-only clearance check doesn't know about it; keep test motions inside regions already used safely
  (home / sleep / horizontal / 50 mm push) unless the user OKs a wider area.

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
    paths, `ArmModel.link_frames()` (FK of every drawn link for the 3D view) · `program.py` motion programs:
    `ProgramBuilder` verbs (`joint_to`/`line_to`/`line_by`/`open_gripper`/`grip`/`check_holding`, no modal state)
    → validated steps → `run(steps, motion)`
  - `cell/` `inventory.py` SQLite · `procedures.py` `Cell`: pick/place + resumable `package_box`
  - `tools/` `teach.py` teach pendant · `demo.py` waypoint demo · `tune.py` servo tuning test · `calibrate.py`
    (servo calibration) · `tray_calibrate.py` hand-guided tray calibration (CLI)
  - `boxes/` packing boxes: `geometry.py` (box.step, capture math, footprints) · `store.py` (orders DB) ·
    `move.py` (box moves) · `pack.py` (whole-order program) · `packer.py` (`TrayPacker`, runs it on the hub's arm)
  - `tray/` parts tray: `dxf.py` ezdxf reader (circles + closed rect loops) · `layout.py` pockets by type + plate
    corner points · `calibration.py` tray→robot rigid fit + checks · `session.py` `CalibrationSession` (dashboard
    calibration, corners/pockets mode) · `goto.py` go-to preview planner · `pick.py` pick planner (`AxisIK`)
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
  up to `max_items_per_order` (10) items per order, any mix; items cycle through the `box_slots` drop spots
  (one spot today: item i → `box_slots[i % len]`). `cfg.stock_slots` = part_slots + tray pockets.
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
  report shows the worst angle per item.
- **Pick test** (plan `.claude/plans/tray_pick.md`; `tray/pick.py`, `POST /api/arm/pick {pocket, confirm_move}`,
  dashboard **Controls** tab → Procedures "Pick…"; same runner/Stop as go-to): open → free move to approach point → line in
  to `grasp_standoff_mm` (0.15 in) off the face (gripper opened only to the part width + `gripper.open_margin_mm`:
  `GripperCfg.open_for`, chord of the jaw swing, `jaw_length_mm` 80 URDF estimate, deg/unit from the calibration;
  part width = type `grip_mm` or the DXF feature; piezo `grip_mm: 12` is an estimate; tool point aimed beside
  the part toward the **fixed jaw** — its inner face is at the tool point, from the meshes — by radius + half margin)
  → grip (miss: open + back out) → pull straight out `pull_mm` (150,
  else `extract_mm` 120) → lift `lift_mm` → hold. Jaws close along gripper_frame x (= type `grip_axis`; per-type
  `min_grip`). A level tool can't pull 12 cm (pull heads at the base): the planner pitches the tool down 0..40° and
  uses the smallest pitch that fits. Reports `pull_twist_deg` (5-DOF yaw change) and `jaw_off_deg` (breadboard
  jaws tilt with the pitch, up to ~40°). Jaw tips stick out ~6 mm past the tool point (open question:
  fingertip tool point + recalibration, see the plan). **No inventory change** (test); drop spot TBD; `Cell` doesn't pick from
  pockets yet. Not yet run on the real arm.

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
  **Delete** (one click, no confirm) allowed for queued/ready/failed/cancelled; **Delete all** (`DELETE /api/orders`,
  click twice within 3 s) removes every such order. Statuses: queued → ready → packing → packed | failed.
- **Per-unit reservations** (plan `.claude/plans/boxes_and_reservations.md`): every item of an open order
  (queued/ready/packing/failed) owns one specific pocket/slot — `reservations` table in the orders DB,
  `UNIQUE(slot)`, all-or-nothing `OrderStore.create(..., stock=shelf.stock)` in one IMMEDIATE transaction
  (lowest-numbered free unit first). SQLite triggers release them on cancel/delete and consume them on packed.
  Pocket marked empty → reservation moves to another free unit or the item goes short (`slot` NULL); marked full /
  restock → `fill_short`. **Locked**: units of a `packing` order can't be changed from the dashboard (409; bulk fill
  skips them). Tray API: `reserved_by`, `locked`; orders carry `slots`. Sim packing consumes exactly those units.
- **Boxes + full order packing** (plan `.claude/plans/boxes_and_reservations.md`, code `packing/boxes/`, sim-tested,
  not run on hardware): supply boxes on the LEFT, tab facing the swivel, box centers **≥ 30.5 cm from the swivel** (`boxes.min_center_mm`;
  Add box refuses closer — the 22 mm level approach/back-out along the tab can't be reached there)
  and ≥ 12 cm apart. **Grip is horizontal**: gripper level, pointing away from the arm along the tab, jaws closing
  up/down on the 2 mm tab plate (fixed jaw under the plate = wrist roll ≈ 0; on top needs roll ~180°, past its
  limit). A level grip only reaches with the grip point ≥ ~25 cm from the swivel. Calibrate = motors off, that
  grip by hand → Calibration tab "Add box". Box move: in level along the tab (22 mm), grip (no width check: plate
  2 mm / wall ~0.5 mm), straight up 30 mm — or higher (60…220 mm) until the carry clears everything — joint-space
  carry checked in **3D** (box block vs other boxes + tray plate/parts + the space under it; no dip; box tilt ≤ 30°),
  straight down to exactly the picked height (box tilt ≤ 5°), release, back out level. Tilts are measured on the
  box. Order (Action, `start --robot` → `TrayPacker`): box → pack spot (355, 0) mm (works for grips 8–20 mm from the wall;
  345 failed deeper grips, 365+ the drops), each reserved part picked + dropped (centered over the opening, jaws above the
  rim, end above the floor; no wall check), box → its supply spot mirrored right. Tray goes on the LEFT.
  `GET /api/boxes`, `POST /api/boxes/capture`, `POST /api/arm/box-move`, `POST /api/orders/{id}/pack-plan` (dry run,
  Orders → Check). Part `length_mm` per tray type = estimates (drop height).
- **Planned-path view**: every dry run (pick, box move, go-to, order Check) returns `path` = tool-point segments
  (`program.tip_path`: free / line / grip / release); a failed plan returns 422 with detail `{message, path,
  fail_m}` (`PathError.point/.steps/.start`, kept through `PathError.wrap` by every planner layer). Both 3D views
  draw it (dashed = free move, blue = straight line, green = grip, orange = release, red = fails here) with a
  legend + Clear (`web/src/arm/plannedPath.ts`, `PathOverlay.tsx`).
- **Real data only** (user asked: no placeholder stock): orders in `packing/data/shop_orders.db`;
  stock from the cell inventory `packing/data/inventory.db` via `shelf.Shelf` (opens the DB per call —
  SQLite connections are per-thread). Inventory starts empty: dashboard **Restock** after filling slots.
  `ShelfSimPacker` consumes real parts so stock stays true. Tray pockets are marked full/empty from the dashboard
  (humans restock; trust that state). Tests/live checks must use a temp copy of `cell.yaml`
  (`packing.tests.test_tray.temp_config`) so they don't write the real databases / calibration.
- **Arm ownership**: `shop/arm_hub.py` `ArmHub` owns the arm in the server process (real with `--robot`, FakeArm
  otherwise; connects on demand, polls ~10 Hz, FK frames). Modes: idle / calibrating / packing / moving (go-to).
  On connect it applies `servo_p` / `servo_accel` (like every other arm user) and it owns the gravity feed-forward
  (`hub.gravity`, real arm only): go-to `Motion`s use it, and holds (motors back on, Stop) command
  `hub.hold_target(pose)` = pose + gravity offset so the arm doesn't sag. New arm users must do the same:
  `arm.tune(...)` + `Motion(..., gravity=gravity_feedforward(...))` on the real arm.
  In-process packing borrows it (`RobotPacker(arm_provider=hub.borrow)`); `shop/goto_runner.py` borrows it for
  go-to moves; a separate `robot --robot` process opens its own arm and posts `/robot/telemetry`. Torque off only
  after a `confirm_hold` from the UI, motion only after `confirm_move`; `/robot/claim` hands out nothing while
  calibrating. Restarting the server drops an unsaved calibration session (points live in memory) and leaves the
  torque as it was (off if calibrating) until Connect.
- Robot agent polls `POST /robot/claim` (returns the in-progress order first → resume), reports
  `done`/`failed`, and **exits on a packing failure** (human checks arm).
- Dashboard = React app (`packing/shop/web/`, served from `static/app/` at `/dashboard/`; old
  `static/dashboard.html` is only a fallback if the build is missing). Tabs: **Orders** (status plate, chat,
  stock + Restock, 2D tray map — click a pocket to toggle full/empty, orders with Action/Delete, live activity via
  SSE `/api/events/stream`, in-memory `events.EventLog`, last 500), **Controls** (`web/src/controls/`: live 3D arm +
  tray, arm connect + joint values, **Motors** Locked/Released via `POST /api/arm/torque {on, confirm_hold}` — idle
  only, release needs the hold confirmation — and **Procedures**: pick from a pocket, dry run → confirm → run, Stop,
  live step/result from `GET /api/arm/goto`) and **Calibration** (calibration only: 3D view, compact arm status,
  Corners/Pockets switch, capture buttons + extra point, live fit, Save/Finish, "Check: go to point" with Stop).
  Both 3D tabs share `web/src/arm/useArmLive.ts`. The old "Preview pose" toggle is gone from the UI
  (`/api/arm/preview` still exists).
  API contract: `.claude/plans/arm_viz.md`.
  3D arm = stick figure (line per segment, arc per joint bend, jaw line) + bright red dot at the tool point. Joints are
  projected into the arm's vertical plane (URDF lift/elbow/wrist origins sit 18.3 mm along their axes — fake jogs
  otherwise); the tool point is the true FK position. Avoid CSS class names that equal event kinds
  (`tool/robot/order/chat/error/system`) — they collide with the activity list styles.
- **Remote-agent MCP** (`shop/mcp_server.py`, FastMCP, mounted by `start`/`serve` at **`/agent/mcp`**, streamable
  HTTP, stateless): `get_inventory` (sku/name/available = stock − open orders) and `create_order` (customer_name,
  items, phone?), `check_order` (status + queue position) and `cancel_order` (received/ready only) — same DBs;
  orders arrive `queued`, source `mcp-remote`, and are **not** auto-dispatched: the robot packs only after Action.
  For the cloud-agent tunnel test. A tunnel to :8000 exposes the whole server (dashboard + arm API, no auth) —
  only `/agent/mcp` checks the token. Flags: `--mcp-token` (or env
  `SHOP_MCP_TOKEN`) → Bearer auth, else open; `--allow-host` (repeatable, wildcards) → Host check, else any host.
  `/mcp` (chat agent's tools) is unchanged.
- Vapi later: put it in front of the same MCP server (Vapi `mcp` tool type) or the old webhook adapter.
  Phone-agent system prompt to paste into Vapi: `packing/shop/vapi_prompt.md` (uses `{{customer.number}}`).

## Conventions
- Every location `L` = poses `L.above` + `L.at`; between stations go via `travel_pose` (`home`).
- Inventory is committed before and after each physical step; `in_gripper` items block runs until resolved.
- SO-101 has 5 body DOF: IK targets must not fix tool yaw (it's tied to `shoulder_pan`). `ArmModel`
  splits R = Rz(azimuth) @ local and only holds/interpolates the local part — keep it that way.
- Joint moves = curved tip path; `{line: X}` / `goto --line` = straight tip path via IK, fully planned
  and validated (reach, joint limits, jumps) before moving.
- Keep new features testable with `FakeArm` + `--sim`; add tests in `packing/tests/`.
- **Workspace safety**: mind the monitor next to the arm (see Hardware facts). Go-to / pick previews must stay
  near the tray and inside regions already used safely.
- **Keep this file current**: whenever a change affects layout, commands, APIs, config, data files or rules here,
  update CLAUDE.md (and the plans it points to) in the same change, without being asked.
- **Prefer a well-fitting library over custom code** for anything complex (math, geometry, file formats,
  UI). Write it ourselves only if no good library fits, or the library has a real downside (huge, unmaintained,
  wrong license, poor fit). Add new deps to `requirements.txt`. E.g. DXF via `ezdxf`, rigid fits via `scipy`.
