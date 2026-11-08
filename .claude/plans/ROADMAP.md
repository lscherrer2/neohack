# Roadmap: state of the world → end goal

Written 2026-11-07; updated after merging `tray-core` with the remote `main` (motion/IK accuracy work) into `main`. **End goal:** a customer orders a part (web chat now, phone via Vapi later)
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
- 🟡 Safety is software-only: confirm dialogs (`confirm_hold`, `confirm_move`), slow moves, following-error stop,
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
  **Calibrated on the real arm 2026-11-07: 2.15 mm RMS.**
- ✅ The table plane is implicit (z = 0).
- 🔴 **Boxes:** no model or calibration yet. `box.step` has the geometry; positions are still taught-pose
  placeholders (`box_in_*`, `pack_1`, `box_out_*`, `box_slot_1` in `cell.yaml`, none taught).
- 🔴 **Drop spot** (a simpler alternative or first step before boxes): not defined.
- **Missing:** a box pose model (like the tray: geometry + a few touched points, or a fixed fixture + taught pose),
  and how the arm grabs a box by its tab.

### 4. Motion primitives 🟡
- ✅ `Motion`: smooth joint moves, `follow()` for dense paths, settle checks, following-error stop, sag
  compensation, stall-detect grip (`grip(min_width)`).
- ✅ Go-to preview (`tray/goto.py` + `shop/goto_runner.py`): plan, validate, confirm, slow move, Stop. The tool
  points along the tray normal as closely as 5-DOF allows. User checked it visually on the real arm.
- **Missing:**
  - An "approach along the normal → grip → extract along the normal" primitive (the go-to plan is half of it).
  - Wrist roll set from the pocket's grip axis: fingers left↔right for most items, up↔down for the breadboard.
  - Per-item grip widths: `grip_width` isn't in the config yet. The breadboard is 2.5 mm, below the global
    `min_part_width` of 3.
  - Collision awareness. Today it's only "approach from the normal, stand-off first"; nothing knows about
    neighboring items or the tray edge.

### 5. Manipulation procedures 🔴 ← **next big piece**
- 🔴 **Pick from a pocket:**
  1. Approach to a stand-off out along the normal, gripper open, wrist roll per grip axis.
  2. Straight line in to the grasp depth (items stick out 10 mm).
  3. Close with stall detection; check the width against the item's `grip_width`.
  4. Pull straight out at least **120 mm** along the normal (the glue stick's length).
  5. Inventory: pocket → `in_gripper` (committed before and after each step, as the cell already does).
- 🔴 **Place:** into a drop spot first, then into a box (box at the pack station; opening above the box; release).
- 🔴 **Failure handling:**
  - Missed grasp → mark the pocket `missing` / empty and try the next pocket of the same item (the cell already
    does this for taught slots).
  - Item dropped during extraction → stop, flag it, a human checks.
- 🟡 Known physics risk (accepted): off-center pockets are gripped tilted 10–20° (no wrist yaw). Watch for
  binding on the long glue stick and the thin breadboard.

### 6. Cell orchestration & inventory 🟡
- ✅ SQLite inventory (`cell/inventory.py`), committed before and after each physical step, resumable after a
  crash, event log.
- ✅ Tray pockets are inventory slots (`cfg.stock_slots`). Humans mark them full/empty on the dashboard.
- ✅ `Cell.package_box`: box to station → fill → ship, resumable. One item per order (`box_slots` has 1 slot).
- 🔴 `Cell` only picks from taught `part_slots` (now empty), **not from tray pockets**. Needs a `pick_tray`
  path and a pocket choice (lowest-numbered full pocket of that item).
- 🔴 The box steps need taught or calibrated box positions (layer 3).

### 7. Orders, queue & robot agent ✅ (mostly)
- ✅ Order DB (`service/orders.py`, `data/shop_orders.db`). Lifecycle:
  queued → ready → packing → packed | failed, plus cancelled and phone call-back statuses (calling / called /
  call_failed) from the parked Vapi service.
- ✅ **Queue is done:** manual dispatch by default (dashboard **Action** → ready), `--auto` option, claim
  returns the in-progress order first (resume), stock reservations, delete/cancel rules.
- ✅ Robot agent (`shop/robot_agent.py`): claim → pack → report; exits on a packing failure; borrows the arm
  from `ArmHub` in-process.
- 🟡 Real packing = `RobotPacker` → `Cell.package_box`, which can't pick from the tray yet (layers 5/6).
  Simulated packing (`ShelfSimPacker`) works end to end and uses up real inventory.
- **Missing:** once layer 5/6 exist, `RobotPacker` uses them. Also a hand-off step ("box at output" → customer
  picked it up → `clear-output`).

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
- **Tests:** 82 sim tests (`.venv/bin/python -m unittest discover -s packing/tests -t .`). New manipulation code
  needs FakeArm tests first (plan validation, inventory transitions, failure paths), then supervised hardware runs.
- **Repo:** `tray-core` and the remote `main` were merged into `main` (2026-11-07). Work on `main` or short
  feature branches from here. `.zed/settings.json` (editor settings) came in from `main`.
- **Docs:** `CLAUDE.md` is kept current (rule). Plans: `tray_layout.md` (tray), `arm_viz.md` (dashboard, arm hub,
  API contract), `shop_slim.md` (orders), `packing_cell.md` (cell), this file (order of work).

---

## Order of work

1. ~~Merge the IK/motion branch~~ (done 2026-11-07). Re-check a go-to preview on the real arm with the new tuning.
2. **Pick one item from one pocket and hold it up** (layers 4–5). ← *next incremental step, details below*
3. Pick → **drop spot** (layer 5): release at a fixed, taught spot; inventory `in_gripper` → packed/dropped.
4. **Boxes** (layers 3, 5, 6): fixture or calibrate box positions, place the item into a box at the pack station.
   Box transfer by the tab can come later.
5. **Wire into orders** (layers 6–7): `Cell` picks tray pockets by item, `RobotPacker` uses it; run a real order
   from chat → packed box, still with manual **Action** dispatch.
6. **Hand-off & notifications** (layer 7): output spot, `clear-output` from the dashboard, "your order is ready".
7. **Vapi** (layer 9): phone ordering through the same MCP tools, call-backs.
8. Hardening (any time): e-stop, collision awareness, path preview in 3D, optional table-touch joint calibration,
   SQLite race fix.

## Next incremental step: "pick test" for one pocket

**Goal:** from the dashboard, pick one item (start with a pocket near the tray's centerline, e.g. `piezo_4` or
`glue_4`, where the tilt is smallest), pull it straight out 120 mm along the normal, and hold it up. The pocket
is marked empty.

Build it the same way as go-to (plan → validate → confirm → slow move → Stop):
- **Config:** per-item `grip_width` (and maybe grasp depth) in `tray.types`; wrist roll from `grip_axis`.
- **Planner** (`tray/pick.py`, extending `tray/goto.py`):
  1. Approach pose at a stand-off with the right wrist roll and the gripper open.
  2. Straight line to the grasp point (about half the 10 mm protrusion).
  3. Grip.
  4. Straight line out 120 mm.
  Validate every step before moving.
- **Runner:** reuse `GotoRunner`'s Stop/hold handling. Inventory: commit `in_gripper` before the grip, `missing`
  on a missed grasp.
- **API/UI:** `POST /api/arm/pick {pocket, confirm_move}` with a dry run first; a "Pick test" button next to
  Go to; the pocket shows empty on the tray map afterward.
- **Tests:** FakeArm with `gripper_block` (grasp succeeds) and without (missed grasp → `missing`, no crash).
- **Done when:** on the real arm, 3 picks in a row from one item type succeed and the inventory matches reality.
