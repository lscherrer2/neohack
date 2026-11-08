# Roadmap: state of the world → end goal

Written 2026-11-07; updated 2026-11-07 with the order-procedure plan (TrayPacker only, see "Next steps"). **End goal:** a customer orders a part (web chat now, phone via Vapi later)
→ the order is queued → the SO-101 picks that part from the calibrated parts tray → puts it in a box → the box is
handed off → the customer is told it's ready → inventory stays true throughout.

The system is a stack: each layer only works if the ones under it do. Status per layer:
✅ done · 🟡 works, known gaps · 🔴 missing.

---

## The stack (bottom → top)

### 0. Hardware & safety 🟡
- ✅ SO-101 follower (Hiwonder kit, STS3215 servos IDs 1–6), driven directly over the Feetech SDK (no LeRobot).
- ✅ Base sits directly on the table (table = z 0). Parts tray clamped, tilted ~69° from flat, faces the arm.
- ✅ Wrist camera available through `camera/cam_server.py` (user runs it; Claude reads frames over HTTP).
- 🟡 Safety is software-only: the API's `confirm_hold` / `confirm_move` (the UI has no confirm dialogs since
  2026-11-07 — one click acts), slow moves, following-error stop,
  Stop button. No physical e-stop.
- ⚠️ **There is a monitor near the arm.** It was hit during a wide pose sweep (2026-11-07). The height-only
  clearance check doesn't know about it. Keep test motions in regions already used safely unless the user OKs a
  wider area, and add it as a keep-out zone once collision awareness exists.
- 🔴 Packing boxes (`box.step`, 76 × 76 × 77 mm with a grab tab) exist physically but aren't placed / fixtured yet.
- **Missing:** a physical e-stop or kill switch; decide where boxes live (supply, pack spot, output).

### 1. Servo driver & joint calibration 🟡
- ✅ `hardware/so101.py`: registers, sync read/write, units (body −100..100, gripper 0..100), calibration JSON
  mirrored in servo EEPROM.
- ✅ Tuning (from `main`, measured by tracing every `Motion` tick): servo P=31, accel 254, velocity feed-forward
  (`lead_s` 0.077), gravity feed-forward from URDF masses (`gravity_gain` 3.21, real arm only), sag integral that
  learns only at rest, `settle_tol` 1. Result: joints in step within ~3 %, lag ~1 unit.
- 🟡 The calibration center pose was eyeballed. After the kinematics fix, readings with the tip on the table sit
  at +5…+10 mm (good enough to move on).
- **Missing (optional):** table-touch joint-offset calibration (touch the table at ~8 poses → scipy fit of
  lift/elbow/wrist offsets). Only needed if pick accuracy turns out to be the bottleneck.

### 2. Kinematics (FK / IK) ✅
- ✅ FK from the official SO-101 URDF. URDF 0° = the `calibrate center` pose (fixed 2026-11-07; it used to be the
  middle of the range, which put the tip 2–4 cm low).
- ✅ Basic placo IK, validated straight-line paths (`ArmModel.line`), 5-DOF handling (no wrist yaw).
- ✅ **IK/motion accuracy work merged** from the remote `main` (2026-11-07): coordinated moves, the same URDF-zero
  fix (done on both branches), gravity and velocity feed-forward. The 50 mm IK line stays within ~5 mm, including at
  45° base rotation. The go-to runner uses the gravity feed-forward on the real arm.
- **Check next:** re-run a go-to preview on the real arm with the new tuning (P=31, feed-forwards). If more IK work
  exists on another branch, merge it the same way.

### 3. World model (where things are) 🟡
- ✅ **Parts tray:** 33 pockets from `part.dxf` (AA ×7, 9V ×5, piezo ×8, glue ×8, breadboard ×5) plus 4 plate
  corners. Tray → robot calibration by touching the corners (dashboard or CLI), 3 mm RMS limit.
  **Calibrated on the real arm 2026-11-07: 2.15 mm RMS** (must be redone once the tray moves to the left).
- ✅ The table plane is implicit (z = 0).
- 🟡 **Boxes** (sim-tested): calibrated by hand-guiding a level grip onto the tab (`packing/boxes/`, orders DB
  `boxes` table, state available/in_use/packed, location supply/pack/output). Supply on the left, pack spot
  (355, 0) mm, output = supply mirrored right. Not yet placed on the real table.
- **Missing:** the monitor as a keep-out zone.

### 4. Motion primitives 🟡
- ✅ `Motion`: smooth joint moves, `follow()` for dense paths, settle checks, following-error stop, sag
  compensation, stall-detect grip (`grip(min_width)`).
- ✅ Go-to preview (`tray/goto.py` + `shop/goto_runner.py`): plan, validate, confirm, slow move, Stop. The tool
  points along the tray normal as closely as 5-DOF allows. User checked it visually on the real arm.
- ✅ Motion programs (`motion/program.py`, 2026-11-07): validated step lists (joint move, line, open, grip,
  check holding; `on_fail` recovery); go-to and pick both run through them.
- ✅ Approach → grip → extract primitive with wrist roll from the grip axis and per-type `min_grip`
  (`tray/pick.py`, plan `tray_pick.md`). Sim only so far.
- **Missing:**
  - Collision awareness. Today it's only "approach from the normal, stand-off first"; nothing knows about
    neighboring items or the tray edge.

### 5. Manipulation procedures 🟡 ← **next: see "Next steps" below**
- 🟡 Whole-order program (`boxes/pack.py` `plan_pack_order`): box supply → pack, per item pick from its reserved
  pocket + drop into the box, box pack → output. Validated before moving; dry run = Orders → Check. Sim only.
- 🟡 **Pick from a pocket** (`tray/pick.py`, plan `tray_pick.md`): all 33 pockets plan on the real calibration
  with the tool pitched 20–40° down. Not yet run on the real arm.
- 🟡 **Box moves** (`boxes/move.py`): level tab grip, direct carry or arc over obstacles, 3D-checked. Sim only.
- 🟡 **Release into the box** (`plan_drop`): side release, jaws closing horizontally, tool level (glue pitched
  ~65°, end first), part center over the box. All 33 pockets plan at the pack spot. Sim only.
- Known physics risk (accepted): off-center pockets are gripped tilted 10–20° (no wrist yaw).

### 6. Orchestration & inventory 🟡
- ✅ SQLite inventory (`cell/inventory.py`) for tray pockets; humans mark pockets full/empty on the dashboard.
- ✅ Per-unit reservations (orders DB): each order item owns a pocket.
- 🟡 `TrayPacker` (`boxes/packer.py`) is **the** packer (user decision 2026-11-07). Each unit is marked used as
  soon as it's out of its pocket; a missed grip marks the pocket empty and stops the order. Sim only.
- ✅ The old `Cell` / `RobotPacker` path was deleted (2026-11-07).

### 7. Orders, queue & robot agent ✅ (mostly)
- ✅ Order DB (`service/orders.py`, `data/shop_orders.db`). Lifecycle:
  queued → ready → packing → packed | failed, plus cancelled and phone call-back statuses (calling / called /
  call_failed) from the parked Vapi service.
- ✅ **Queue is done:** manual dispatch by default (dashboard **Action** → ready), `--auto` option, claim
  returns the in-progress order first, stock reservations, delete/cancel rules.
- ✅ Robot agent (`shop/robot_agent.py`): claim → pack → report; exits on a packing failure; with `start --robot`
  it packs through `TrayPacker` on the hub's arm. Simulated packing (`ShelfSimPacker`) uses up real inventory.
- ✅ "Picked up" for packed boxes at output (Calibration → Packing boxes). Boxes move one way only.
- **Missing:** customer notification (with Vapi).

### 8. Interfaces: MCP, chat, dashboard 🟡
- ✅ MCP server at `/mcp` with `list_products`, `place_order`, `check_order`, `cancel_order`. Stock = full tray
  pockets, catalog names from the tray types.
- ✅ Ordering agent (Claude Haiku) in web chat (dashboard) and terminal chat, through the same MCP tools.
- ✅ Dashboard (React, the dev/test hub):
  - **Orders tab:** status, chat, stock, tray map, orders, live activity.
  - **Calibration tab:** live 3D arm (stick figure plus red tool point), tray, preview poses, Corners/Pockets
    calibration, go-to preview with Stop.
- **Missing:**
  - A "Pick test" / "Place test" button on the dashboard (same plan → confirm → move → Stop pattern as go-to) to
    develop layer 5 safely.
  - Box positions in the 3D view.
  - Showing the planned path in 3D before confirming a move.
  - Small fixes: the 3D target label shows `corner_bl` instead of "Bottom-left corner"; a one-off SQLite
    thread-race error on `/api/state` (`sqlite3.InterfaceError`).

### 9. Voice: Vapi 🔴 (last)
- 🟡 Groundwork exists: the parked `service/` has a Vapi webhook adapter and call-back worker
  (`VAPI_SETUP.md`). The plan is to put Vapi in front of the same MCP server (Vapi `mcp` tool type) or reuse the
  old adapter.
- **Missing:**
  - A public URL (tunnel) and Vapi assistant config.
  - Call-back when an order is packed (statuses already exist).
  - An end-to-end phone test.
  - Secrets handling (keys in `.env.local`, never printed).

### Cross-cutting
- **Tests:** 142 sim tests (`.venv/bin/python -m unittest discover -s packing/tests -t .`). New manipulation code
  needs FakeArm tests first (plan validation, inventory transitions, failure paths), then supervised hardware runs.
- **Repo:** `tray-core` and the remote `main` were merged into `main` (2026-11-07). Work on `main` or short
  feature branches from here. `.zed/settings.json` (editor settings) came in from `main`.
- **Docs:** `CLAUDE.md` is kept current (rule). Plans: `tray_layout.md` (tray), `arm_viz.md` (dashboard, arm hub,
  API contract), `shop_slim.md` (orders), `packing_cell.md` (cell), this file (order of work).

---

## Order of work

1. ~~Merge the IK/motion branch~~ (done 2026-11-07).
2. ~~Pick test from one pocket~~ (planned + on the dashboard; hardware run is part of step 3's bring-up).
3. **One order procedure on TrayPacker** ← *now; details in "Next steps" below* (cleanup, one-way box flow,
   side release, per-item inventory, hardware bring-up).
4. **Hand-off & notifications** (layer 7): "picked up", "your order is ready".
5. **Vapi** (layer 9): phone ordering through the same MCP tools, call-backs.
6. Hardening (any time): e-stop, collision awareness (monitor keep-out), optional table-touch joint calibration,
   SQLite race fix.

## Next steps: one order procedure (TrayPacker), 2026-11-07

**Goal:** one procedure for a whole order, run by `TrayPacker` (`packing/boxes/packer.py`, program from
`packing/boxes/pack.py`): (1) take an empty box from its supply spot (left), (2) move it to the pack spot (middle),
(3) pick each ordered part from its reserved pocket and drop it into the box, (4) move the packed box to its output
spot (right) for pickup. Most of this already plans and runs in sim (`boxes_and_reservations.md`); what's left is
cleanup, a new release pose, inventory bookkeeping and the first hardware runs.

### Decisions (user, 2026-11-07)
- **Demo quality, not production.** Assume moves succeed. No resume, no retries, no handling of bad states.
- **TrayPacker is the only packer.** Delete the old `Cell` path.
- **Release above the rim is fine** (a drop, not a gentle place). Turn the jaws to the side for the release
  (see step 3).
- **Boxes only move left → right.** Supply (left) → pack (middle) → output (right, for pickup). They never
  come back. New boxes come in only through Calibration → "Add box".
- **Only failure case to handle:** the gripper closes on nothing → mark that pocket empty and stop the order.

---

**Status 2026-11-07:** steps 1–5 and 7 are done in sim (branch `order-procedure`); step 6 (hardware) is next.
Notes: `robot --robot` exits with a hint (packing on the real arm = `start --robot`); `/api/boxes/{id}/reset` was
removed (new boxes only via Add box); refusals before the arm moves put the order back to "received". Pack spot:
355 kept; 365 and 355 ± 50 y also plan all 33 pockets; from 380 glue only fits level (lands across the rim).
The cross-process telemetry path (`POST /robot/telemetry`, `http_publisher`, `ArmHub.external`) was removed too:
`serve` without `--robot` now owns a simulated arm like `start`.

### 1. Delete the old Cell path
Files that use it: `packing/cell/procedures.py`, `packing/service/worker.py` (`RobotPacker`),
`packing/service/__main__.py`, `packing/shop/__main__.py` (`make_packer`), `packing/cli.py`,
`packing/tests/test_packing.py`.
- [x] Delete `cell/procedures.py` and `RobotPacker`. `make_packer` with `--robot` → always `TrayPacker` (it
      needs the hub, so only in-process: `start --robot` / `serve --robot`). `robot --robot` as a separate
      process goes away; `robot` without `--robot` stays as the sim packer.
- [x] CLI: remove `pack`, `move-box`, `set-box`, `clear-output`, `teach-all`/`check` (if they only exist for
      Cell poses). Keep `teach`/`goto`/`where`/`home`/`release`/`tune`/`demo`/`tray`/`tray-calibrate`/`status`/
      `events`/`set-part`/`restock`.
- [x] `cell/inventory.py` stays (the shelf uses it for tray pockets). Drop its box part (box table/`restock` of
      "box supply spots"): boxes live only in the orders DB `boxes` table.
- [x] `cell.yaml`: remove the Cell-only placeholders (`box_in_*`, `pack_1`, `box_out_*`, `box_slot_1`,
      `box_slots`, `pack_station`, `part_slots`, if nothing else reads them). Check `config.py` and
      `max_items_per_order` (which uses `box_slots` today).
- [x] Parked `packing/service/`: keep `orders.py` and `vapi.py` (the shop uses them). Delete or stub anything that
      only fed `RobotPacker`.
- [x] Tests: drop or port `test_packing.py`. The whole suite passes.

### 2. One-way box flow
- [x] Box moves only go forward: supply → pack → output. Remove `to: supply` from `/api/arm/box-move` and the
      dashboard Move-a-box control. Remove `POST /api/boxes/{id}/reset` (or keep it only as a manual "this box
      is empty at supply again" fix; decide while doing it).
- [x] **Pack spot taken:** if another box is already at `pack`, refuse to start an order with a plain message
      ("box N is still at the pack spot"), not a collision error.
- [x] **Output spot:** each box's output spot = its supply spot mirrored right (one per box, so finished boxes
      never land on each other). Keep that. Packed boxes left at output stay obstacles for later moves (they are
      already).
- [x] **Picked up:** a "Picked up" button on a packed box (Orders or Calibration tab) deletes the box record, so
      it stops being an obstacle (`DELETE /api/boxes/{id}` already exists; just label and place it).
- [x] No empty box at supply → the order isn't started; message "add a box" (exists: `BoxError`).

### 3. Release with the jaws turned to the side
Today `plan_drop` points the gripper straight down over the box. That's only reachable up to ~10 cm above the
table, which limits the pack spot and the drops (365 mm+ already fails).
- [x] New release pose: the tool does **not** have to point down. Roll the wrist so the jaws close
      **horizontally** (gripper x level), so when they open there is nothing under the part and it falls straight
      down. Tool level or tilted down, whichever reaches; prefer level/slightly down for range.
- [x] Place the part's center (jaw point + half its length along the tool axis) over the box opening, jaws/part
      above the rim + `RIM_CLEAR_MM`. Long parts (glue ~120 mm) stick out past the 76 mm opening; for a demo, the
      center over the opening is enough (it tips in).
- [x] The wrist roll may have to turn ~90° from the pick roll while holding the part. `wrist_roll` range is
      ~337° (raw 142–3972), so it fits; plan the roll during the free move to the box.
- [x] Keep `check_clear_of_box` (the part/arm must not hit the box on the way in).
- [x] Sim check: all 33 pockets plan at pack spot 355. Then see whether the pack spot can move (more room).
- [x] Tests in `packing/tests/` (jaw axis level at the release, part center over the opening, above the rim).

### 4. Mark parts removed as they're taken
Today `shelf.consume_slots(order.slots)` runs once, after the whole order.
- [x] After each successful grip (the step after `check_holding` / pull-out): mark that pocket's part used
      (`consume_slots([slot])`), so the tray map and stock update live. Drive it from `run(..., on_step=...)`
      by step label/index (planner returns which step finishes item i).
- [x] Order → `packed` and box → `packed` at `output` at the end (already).
- [x] Event-log line per item ("item 2/3: 9V from 9v_1 → box 1").

### 5. Empty pocket = mark empty and stop
- [x] If the grip closes on nothing (`check_holding` fails; the plan's `on_fail` already opens and backs out):
      mark the pocket **empty** (`shelf.set_slot(slot, False)`; the reservation then moves or goes short by
      itself), log it, stop the order → `failed`. Whatever already happened (box at the pack spot, parts in it)
      stays as is; a person sorts it out.
- [x] Test with `FakeArm` without `gripper_block` (missed grip → pocket empty, order failed, no crash).

### 6. Hardware bring-up (supervised, user says "go" before each motion)
- [ ] Tray on the LEFT, recalibrate (Calibration tab, corners).
- [ ] Add the boxes (supply on the left, centers ≥ 30.5 cm from the swivel, ≥ 12 cm apart).
- [ ] Measure part lengths (`length_mm` per tray type; estimates today) and fix `cell.yaml`.
- [ ] Dry run: Orders → Check on a 1-item order; look at the planned path.
- [ ] Piece by piece on the real arm: box move supply → pack; one pick (Controls → Pick…); one release over the
      box; box move pack → output.
- [ ] One full 1-item order via Action, then a 3-item order.
- [ ] Mind the monitor near the arm: watch the first runs of every new move.

### 7. Docs
- [x] Update `CLAUDE.md` (layout, CLI, shop/boxes sections), `ROADMAP.md` (layers 5–7),
      `boxes_and_reservations.md` (side release, one-way flow), `packing/README.md`.

### Not doing (demo)
Resume after a crash, retries / other units after a miss, dropped-part detection, pause/continue on Stop,
volume check for 10-item orders, customer notification (later, with Vapi).
