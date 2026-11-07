# Plan: live 3D view of the arm + calibrated tray on the dashboard

Status (2026-11-07): **backend built** (branch `tray-core`): `ArmModel.link_frames`, `hardware/telemetry.py`,
`shop/arm_hub.py`, `tray/session.py`, arm/calibration/telemetry endpoints, `RobotPacker(arm_provider|telemetry)`,
shop `--robot` on `serve`; sim hub saves `sim_tray_cal.json`; claim returns nothing while calibrating.
Tests: `packing/tests/test_arm_view.py`. React app: in progress. Depends on the tray work in `tray_layout.md`
(DXF reader + calibration) for the tray part. Boxes / drop spot are added later.

## Goal
A 3D scene on the dashboard showing, in the robot base frame:
- the arm in its **current** pose (live, ~10 Hz),
- the **tray** at its **calibrated** pose, pockets colored full/empty from inventory,
- later: box supply / pack station / output boxes, drop spot.

## Current constraints (found in code)
- The arm is only open **during a pack**: `RobotPacker.pack()` (`packing/service/worker.py`) opens
  `RealArm` per order and closes it after. When idle, nothing reads the arm → no live pose.
- Only one process may own the serial port; the robot agent may run in the server process
  (`start`) or separately (`robot`), and talks to the server only over HTTP.
- `ShelfSimPacker` (default sim) has no arm at all.
- URDF meshes were stripped (`packing/urdf/so101_new_calib.urdf`, kinematics only) → no visual
  geometry for a realistic arm.
- placo FK is available in the server process (`ArmModel.fk`, per-link frames via placo).

## Design

### Telemetry (backend)
- `packing/hardware/telemetry.py`: `TelemetryArm(arm, post)` wraps any `Arm`; on `read()` it keeps
  the latest joints and posts them (throttled ~10 Hz, non-blocking, drops on error) to
  `POST /robot/telemetry {joints, torque, source, t}`.
- Wrap the arm in: robot agent (`RobotPacker`), `teach`, `tray-calibrate` (live view while
  hand-guiding with motors off — useful for checking the reference points).
- Robot agent keeps the arm **open for its lifetime** (not per order) and reads it ~5 Hz while idle,
  so the view stays live between orders. (Changes `RobotPacker` lifecycle; torque stays on as now.)
- Server: stores latest telemetry; computes FK for every link frame (placo) → `GET /api/scene`
  returns `{arm: {joints, frames: {link: 4x4}, tip, stale_s, source}, tray: {...}}`; pushes
  updates over the existing SSE stream as a throttled `arm` event (not in the activity log —
  separate channel so it doesn't flood the 500-event buffer).
- Sim: `--sim` robot agent uses `FakeArm` behind the same wrapper so the view animates in sim.

### Tray in the scene
- From `tray_layout.md`: pockets (DXF) + `data/tray_cal.json` (R, t) → server sends plate outline
  + pocket centers/shapes already transformed to the robot frame, plus pocket state.
- Not calibrated → show "tray not calibrated" instead of guessing.

### Frontend
- Vite + React + TypeScript + `three` / `@react-three/fiber` / `@react-three/drei` (OrbitControls,
  grid, labels).
- Arm drawn from FK link frames as simple capsules/boxes per link + gripper jaws + tip marker
  (meshes optional later).
- Robot frame → three.js: z-up handled by a root rotation; units meters; table grid at z=0.
- Source in `packing/shop/web/`, built into `packing/shop/static/app/`, served by FastAPI at
  `/dashboard`. Port the existing dashboard (status plate, chat, shelf, orders, activity) into the
  React app so there is one UI.

### Tests
- Telemetry wrapper (throttle, failure-tolerant) with FakeArm; `/robot/telemetry` + `/api/scene`
  (FK frames shape, stale handling, uncalibrated tray); frontend: typecheck + build in CI step.

## Decisions (2026-11-07)
1. Port the **whole dashboard** to React + TS (Vite, three / @react-three/fiber / drei).
2. **Both**: `npm install && npm run build` in `packing/shop/web/` works for anyone, and the built
   output in `packing/shop/static/app/` is committed so a Python-only run still works.
3. Arm drawn with **simple shapes** from FK (no meshes).
4. Build order: **tray core first** (`tray_layout.md`: DXF reader, calibration, pocket inventory),
   then telemetry + scene API, then the React app.

## Calibration tab + API contract (2026-11-07, user request)
UI = React app with two tabs: **Orders** (everything the old dashboard had + 2D tray map where clicking a
pocket toggles full/empty) and **Calibration** (3D view + calibration controls). The user wants buttons
"calibrate bottom-left / bottom-right / ..." plus a menu to capture any pocket.

### Backend pieces
- `packing/shop/arm_hub.py` `ArmHub`: the server's single owner of the arm. `connect()` opens RealArm
  (`--robot`) or FakeArm (sim); a poll thread reads joints ~10 Hz and computes FK link frames; `borrow()`
  lends the arm to in-process packing (wrapped so its reads still publish telemetry); torque on/off;
  `external()` takes telemetry from a separate robot process (`POST /robot/telemetry`).
- `packing/hardware/telemetry.py` `TelemetryArm`: wraps an Arm, publishes joints (throttled) on read().
- `packing/tray/session.py` `CalibrationSession`: captured points, live fit preview, save.
- Reference point labels come from pocket position: y below tray center = "Bottom" (nearest the arm),
  x below center = "left" → aa_1 "Bottom-left", aa_7 "Top-left", breadboard_1 "Bottom-right",
  breadboard_5 "Top-right".

### Endpoints (JSON)
ArmState = {
  "connected": bool, "source": "sim"|"real"|"remote"|null, "torque": bool|null,
  "mode": "idle"|"calibrating"|"packing", "stale_s": number|null, "error": string|null,
  "joints": {shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper} | null,  // units
  "frames": {"<link>": 4x4 row-major number[][]} | null,  // robot base frame, meters, z up
  "tip": [x, y, z] | null                                  // meters
}
  links in "frames", base → tip: base_link, shoulder_link, upper_arm_link, lower_arm_link, wrist_link,
  gripper_link, gripper_frame_link (tool point), moving_jaw_so101_v1_link (jaw; opening approximate)
- GET  /api/arm                → ArmState
- GET  /api/arm/stream         → SSE, `data: <ArmState JSON>` ~10 Hz
- POST /api/arm/connect        → ArmState; 409 {"detail": msg} if busy / port error
- POST /api/arm/disconnect     → ArmState (torque stays on; refused while calibrating/packing)

CalSession = {
  "active": bool,                       // motors off, capturing
  "reference_points": [{"pocket": "aa_1", "label": "Bottom-left", "captured": bool}],
  "points": [{"pocket", "label": string|null, "tray_mm": [x, y], "robot_m": [x, y, z], "residual_mm": number|null}],
  "fit": null | {"ok": bool, "error": string|null, "rms_mm": number|null, "tilt_deg": number|null,
                 "warnings": string[], "grip_angles": {"<sku>": deg}, "matrix": 4x4|null},
  "saved": null | {"created": string, "rms_mm": number}
}
- GET    /api/calibration                    → CalSession
- POST   /api/calibration/start  {"confirm_hold": true}   → connects if needed, torque OFF (arm goes limp)
- POST   /api/calibration/capture {"pocket": "aa_1"}      → reads tip now (averaged), adds/replaces point
- DELETE /api/calibration/points/{pocket}
- POST   /api/calibration/save                → saves fit (409 if fit not ok) → CalSession
- POST   /api/calibration/finish {"confirm_hold": true}   → torque ON at current pose, session cleared
  400 if confirm_hold missing; 409 when the arm is packing / not available.
- Tray geometry/state: existing GET /api/tray (outline, pockets with shape/full/robot_m, calibration.matrix),
  POST /api/tray/pockets/{id} {"full"}, POST /api/tray/fill {"full", "sku"?}.

### Frontend
- `packing/shop/web/` Vite + React + TS (+ three, @react-three/fiber, @react-three/drei). Vite `base: "/dashboard/"`,
  `build.outDir: "../static/app"` (committed). FastAPI serves it at `/dashboard`. Dev: `npm run dev` proxies
  `/api`, `/robot`, `/orders`, `/health` to :8000.
- 3D: robot z-up → wrap scene in a group rotated -90° about x. Arm = capsules between consecutive link
  origins + small boxes for gripper/jaw + tip sphere. Tray = thin rounded plate placed by
  `calibration.matrix` (tray mm → m), pockets drawn on its face, full pockets show a 10 mm item stub.
  Uncalibrated: draw the tray ghosted at a default spot in front of the arm with a "not calibrated" label.
  Captured points = spheres at robot_m, highlighted target pocket = the one being captured.
