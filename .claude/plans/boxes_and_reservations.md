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
