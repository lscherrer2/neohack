# Boxes + per-unit stock reservations (2026-11-07)

User decisions (this session): reservations assign specific pockets to orders; packing locks them; boxes are
calibrated by hand-guiding the closed jaws onto the tab; box yaw = tab facing the arm's swivel (pan axis); boxes
sit to the LEFT of the arm, roughly along the forward direction, never overlap; ~2 boxes, identical (`box.step`);
a box is picked up by its tab and moved to the middle of the workspace; DB tracks a box as available / in use /
packed (packed = done). UI: Boxes section on the Calibration tab + boxes in the 3D view.

## 1. Reservations (orders DB, `service/orders.py`)
- Table `reservations(order_id, item, sku, slot NULL, UNIQUE(slot))` — one row per order item; `slot` = the
  pocket / part slot holding that unit; NULL = short (no free unit right now). UNIQUE(slot): a unit belongs to
  at most one order.
- `OrderStore.create(..., stock=fn)`: `stock()` → {sku: [full slots, preferred first]}. Inside one IMMEDIATE
  transaction: free = full − reserved; take the lowest-numbered free slot per item; none → OrderError (nothing
  written). Atomic across threads and processes.
- Released on cancel / delete; consumed (rows dropped) when packed. Failed orders keep theirs until deleted.
- Pocket marked empty while reserved → `reassign(slot)`: move to another free unit of that sku, else NULL
  (short). Pockets marked full / restock → `fill_short()` gives short items a unit.
- **Lock**: units reserved by a `packing` order can't be changed from the dashboard (409); bulk fill skips them.
- In stock = full units − reserved units. Legacy open orders without rows are backfilled on server start.
- Sim packer consumes exactly the reserved units. (The real `Cell` doesn't pick from tray pockets yet.)
- Dashboard: tray map shows "reserved #id" / locked pockets; orders show their pockets.

## 1b. Status: reservations DONE (2026-11-07), commit 9ad42fca.

## 2. Boxes (`packing/boxes/`) — DONE in sim (2026-11-07), not run on hardware
- Geometry from `box.step` (m, box frame: origin bottom center, z up, tab along −y): body 76.2 × 76.2 × 77 mm,
  open top, wall 0.8 mm; tab out of the −y wall: x ±10.4 mm, y −38.1 … −61.0 mm, z ≈ 60 … 77 mm.
- Capture: motors off, jaws closed on the tab → FK tool pose. Box yaw: the tab (box −y) points at the pan
  axis; box position = tool point + offset (tab grip point → box origin) rotated by that yaw; z checked against
  the table. Stored per box with the capture joints (= the grasp pose for picking it up).
- Checks: box must be to the left (y > 0), body must not overlap another box / the arm base / the tray.
- DB: `boxes` table (state: available | in_use | packed, order_id, pose, created).
- Later in this plan: "move box to the pack spot" program (grasp = capture pose, lift, carry to the middle of
  the workspace; with 5-DOF the tab keeps facing the swivel).

### As built
- `geometry.py`: tab = 12.6 mm wide plate (x ±6.3) on the −y wall top edge (z 75–77) out to y −61 + rib to z 38.5.
  Capture (`box_from_grasp`): the fixed jaw's face is at the tool point, tab center 6.3 mm toward tool −x; box lies
  further out radially; refused if the gripper is > 25° off vertical or the box bottom > 20 mm off the table.
  `obstacles()` = footprints of the other boxes + the calibrated tray (table plane).
- `store.py` (orders DB `boxes`): available | in_use | packed, location supply | pack | output, `tool_in_box` (the
  calibration grasp in the box frame — the grasp is rigid, so set-down = same grasp, same height).
- `move.py`: open → straight down onto the tab → grip → straight up `carry_mm` → **joint-space carry** checked sample
  by sample (never dips below half the lift, ≤ 12° tilt, footprint swept vs obstacles) → straight down to exactly the
  picked height → release → jaws up. AxisIK targets (5-DOF + 7.9 mm tool offset); reach-down IK seeds use the model's
  pan sign (left = negative pan). Refuses set-downs tilted > 5°.
- **Wrist limit**: pointing straight down, wrist_flex stops ~10 cm above the table near the base → `carry_mm` 30 and
  pack spot (260, 0) mm (2.8° set-down; 220 gave 7°). Boxes ≥ ~16 cm from the pan axis can be set down level at their
  mirrored output spot; centers ≥ 12 cm apart in a row (the outer tab points at the inner box).
- `pack.py` `plan_pack_order`: box supply → pack, retreat 12 cm above the rim, per item pick (reserved pocket) +
  drop (part hangs `length_mm` past the jaws — per-type ESTIMATES in cell.yaml; jaws ≥ rim + 12 mm, part centered,
  whole part inside the walls and above the floor) + retreat, box pack → output. Everything near the box at the pack
  spot is checked against its volume (jaws, wrist, held part). With the tray straight ahead the pull-outs swept
  through the pack spot (0/33); the user is moving the tray to the LEFT (then all 33 plan in the sim layout).
- `packer.py` `TrayPacker`: used by `start --robot` for in-process packing (Action → robot agent). Plans, then runs
  on the hub's arm; on success consumes the reserved units and marks the box packed at output. Failure → order
  failed, agent stops, a person checks the box (dashboard Reset/Remove).
- API: `/api/boxes` (+ capture, delete, reset), `/api/arm/box-move`, `/api/orders/{id}/pack-plan` (dry run).
  UI: Calibration → Packing boxes (Add box, list, Reset/Remove); boxes + pack spot in both 3D views; Controls →
  Move a box; Orders → Check (dry run per order).

### Open
- Hardware: calibrate the moved tray, add the boxes, Check an order, then a supervised first run.
- Measure `length_mm` per part type and the tab grip estimate (`grip_from_top_mm` / `grip_from_wall_mm`).
- Resume after a mid-order failure is partial (box location is recorded once it's down at the pack spot).

### Change 2026-11-07 (later): horizontal grip
User: the gripper is HORIZONTAL — level, pointing away from the arm along the tab — and the jaws close up/down on
the tab plate (parallel to the desk); approach level along the tab. (Box 2 had been calibrated on a wall to test,
which made the vertical-grip planner report a 12° carry tilt.)
- Capture (`box_from_grasp`): gripper ≤ 25° off level, pointing outward along the tab, jaws ≤ 35° off vertical;
  plate mid-plane = tool point − x·1 mm; box bottom = plate mid − 76 mm. `grip_from_wall_mm` 8 (estimate).
- Move: in level along the tab `approach_mm` 22 → grip (min width 0: user — thin plate/wall, a width test can't
  tell a miss) → up `carry_mm` 30 → joint carry → down to the exact grasp height → release → back out level 22 mm.
  Tilt checks use the BOX tilt (`box_tilt_deg`): carry ≤ 30° (user: tilting is fine), set-down ≤ 5°.
- Reach (sim): a level outward grip needs the grip point ≥ ~25 cm from the swivel (17–25 cm: unreachable; closer:
  only by folding over, 30–50° tilted). Jaw-on-top (tool x up) hits the wrist_roll limit (20° off) → jaw under.
  Pack spot 345 mm: grip point 26 cm out, and drops over the box still reachable (at 360 they weren't above 13 cm).
  Back-out longer than ~22 mm at the pack spot ran into the 25 cm limit.
- Drop: wall check removed (user: walls ~0.5 mm); part centered over the opening, jaws ≥ rim + 12 mm, end above
  the floor.
- Sim layout used in tests: tray rotated 60° left at 33 cm, supply box at (316, 160) mm → all 33 pockets plan.

### Change 2026-11-07 (later): 3D collision checks + vertical space
User: "it keeps saying it will hit the parts tray, but it has so much room to go over it". Cause: the carry check
compared 2D table footprints (a leftover from the fixed 30 mm lift), so any xy crossing counted as a hit, and the
lift height was fixed. Now: `geometry.Block` + separating-axis `blocks_intersect`; obstacles = other boxes (body +
tab block) and the tray (`tray_blocks`: plate + protrusion, plus a stand block from the desk up to the plate's
lowest point). The carried box is checked as a 3D block (10 mm clearance) at 30 samples. `plan_box_move` tries the
lift `carry_mm` (30) first, then 60/100/140/180/220 mm until the carry clears; landing + capture overlap checks
are 3D too. Reach (sim): a level grip reaches tool heights up to ~33 cm at grip radii 26–30 cm, so the box bottom
can go ~25 cm above the desk.

