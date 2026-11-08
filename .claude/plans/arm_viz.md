# Plan: live 3D view of the arm + calibrated tray on the dashboard

Status (2026-11-07): **built and in use on the real arm** (branch `tray-core`). Backend: `ArmModel.link_frames`,
`hardware/telemetry.py`, `shop/arm_hub.py`, `tray/session.py` (corners/pockets mode), `tray/goto.py` +
`shop/goto_runner.py` (go-to preview), arm/calibration/telemetry/preview/go-to endpoints,
`RobotPacker(arm_provider|telemetry)`, shop `--robot` on `serve`; sim hub saves `sim_tray_cal.json`; claim returns
nothing while calibrating. React app (`packing/shop/web/`): Orders + Calibration tabs, built into `static/app/`.
Tests: `packing/tests/test_arm_view.py`. First real tray calibration saved 2026-11-07 (2.15 mm RMS, tray tilted 69°). Depends on the tray work in `tray_layout.md`
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
  "mode": "idle"|"calibrating"|"packing"|"moving", "stale_s": number|null, "error": string|null,
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
- POST /api/arm/torque         {"on": bool, "confirm_hold": bool} → ArmState. Controls tab Motors switch: lock
                                 (hold current pose + gravity FF) / release (needs confirm_hold). Idle only (409 while
                                 calibrating / moving / packing).
- POST /api/arm/pick           {"pocket", "confirm_move"} → {"plan", "calibration", "motors_will_turn_on", "started"}
                                 (Controls tab Procedures; plan in .claude/plans/tray_pick.md)
- GET  /api/boxes              → {"boxes": [{id, state available|in_use|packed, location supply|pack|output,
                                 order_id, pose 4x4 at its location, report}], "pack_spot_mm": [x, y], "geometry"}
- POST /api/boxes/capture      motors off, jaws closed on a box tab (gripper down) → adds that box (BoxesState);
                                 409 motors on / 422 not a plausible tab grasp, on the right, or overlapping
- DELETE /api/boxes/{id}       remove (409 while in use);  POST /api/boxes/{id}/reset → empty at its supply spot
- POST /api/arm/box-move       {"box_id", "to": supply|pack|output, "confirm_move"} → {"plan", "motors_will_turn_on",
                                 "started"}; same runner / Stop / GET /api/arm/goto state; records the new location
- POST /api/orders/{id}/pack-plan  dry run of packing the order (box in, picks + drops, box out) → {"plan": {box_id,
                                 items: [{pocket, sku, pitch_deg, drop_tilt_deg}], step_count}} or 422 with the reason
- Tray pockets (GET /api/tray) also carry "reserved_by" (order id) and "locked" (its order is being packed);
  orders carry "slots" (reserved unit per item, null = short). Plan: boxes_and_reservations.md
- GET  /api/arm/preview        → {"poses": [{"name", "label"}]}; GET /api/arm/preview/{name} → ArmState
                                 (no longer used by the dashboard since the Controls tab, 2026-11-07)
                                 (source "preview": FK of a generic pose, no arm needed; "home" = calibration center)
- POST /api/arm/goto {"point", "confirm_move": false, "standoff_mm": 12.7}
                               → {"plan": {point, point_robot_m, standoff_mm, approach_mm, tilt_deg, travel_units,
                                  line_mm, approach_pose, end_pose}, "calibration": "live fit"|"saved calibration",
                                  "motors_will_turn_on": bool, "started": bool}
                                 confirm_move=false = dry run (plan only); true = start the move (mode "moving").
                                 409 not connected / busy / no calibration; 422 unreachable; 404 unknown point.
- GET  /api/arm/goto           → {"running": bool, "last": {"plan", "status": running|done|stopped|failed, "error"}}
- POST /api/arm/stop           → {"stopping": bool}  (halts at the next control tick, holds)
- POST /api/arm/release {"confirm_hold": true} → ArmState (motors off again during calibration, after a go-to)

CalSession = {
  "active": bool,                       // capturing (motors off unless a go-to turned them on)
  "mode": "corners"|"pockets",          // which reference points the capture buttons use
  "reference_points": [{"pocket": "corner_bl", "label": "Bottom-left corner", "captured": bool}],
                                        // "pocket" = point id: corner_bl/tl/br/tr or a pocket id ("aa_1", label "Bottom-left")
  "points": [{"pocket", "label": string|null, "tray_mm": [x, y], "robot_m": [x, y, z], "residual_mm": number|null}],
  "fit": null | {"ok": bool, "error": string|null, "rms_mm": number|null, "tilt_deg": number|null,
                 "warnings": string[], "grip_angles": {"<sku>": deg}, "matrix": 4x4|null},
  "saved": null | {"created": string, "rms_mm": number}
}
- GET    /api/calibration                    → CalSession
- POST   /api/calibration/start  {"confirm_hold": true}   → connects if needed, torque OFF (arm goes limp)
- POST   /api/calibration/mode {"mode": "corners"|"pockets"} → CalSession (captured points are kept)
- POST   /api/calibration/capture {"pocket": "corner_bl"} → reads tip now (averaged), adds/replaces point
                                                            (409 while the motors are on)
- DELETE /api/calibration/points/{pocket}
- POST   /api/calibration/save                → saves fit (409 if fit not ok) → CalSession
- POST   /api/calibration/finish {"confirm_hold": true}   → torque ON at current pose, session cleared
  400 if confirm_hold missing; 409 when the arm is packing / not available.
- Tray geometry/state: existing GET /api/tray (outline, pockets with shape/full/robot_m, corners [{id, label, x, y}],
  calibration.matrix),
  POST /api/tray/pockets/{id} {"full"}, POST /api/tray/fill {"full", "sku"?}.

### Frontend
- `packing/shop/web/` Vite + React + TS (+ three, @react-three/fiber, @react-three/drei). Vite `base: "/dashboard/"`,
  `build.outDir: "../static/app"` (committed). FastAPI serves it at `/dashboard`. Dev: `npm run dev` proxies
  `/api`, `/robot`, `/orders`, `/health` to :8000.
- 3D: robot z-up → wrap scene in a group rotated -90° about x. Arm (as built, after user feedback) = stick figure:
  a line per segment through the joints projected into the arm's vertical plane, an arc per joint bend, a ring at
  the pan joint, the moving jaw as a V with the fixed finger, a bright red dot at the tool point; base = square on
  the table centered on the pan axis. Plate corners get small cross markers. Tray = thin rounded plate placed by
  `calibration.matrix` (tray mm → m), pockets drawn on its face, full pockets show a 10 mm item stub.
  Uncalibrated: draw the tray ghosted at a default spot in front of the arm with a "not calibrated" label.
  Captured points = spheres at robot_m, highlighted target pocket = the one being captured.
