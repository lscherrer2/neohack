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

## 2. Boxes (`packing/boxes/`)
- Geometry from `box.step` (m, box frame: origin bottom center, z up, tab along −y): body 76.2 × 76.2 × 77 mm,
  open top, wall 0.8 mm; tab out of the −y wall: x ±10.4 mm, y −38.1 … −61.0 mm, z ≈ 60 … 77 mm.
- Capture: motors off, jaws closed on the tab → FK tool pose. Box yaw: the tab (box −y) points at the pan
  axis; box position = tool point + offset (tab grip point → box origin) rotated by that yaw; z checked against
  the table. Stored per box with the capture joints (= the grasp pose for picking it up).
- Checks: box must be to the left (y > 0), body must not overlap another box / the arm base / the tray.
- DB: `boxes` table (state: available | in_use | packed, order_id, pose, created).
- Later in this plan: "move box to the pack spot" program (grasp = capture pose, lift, carry to the middle of
  the workspace; with 5-DOF the tab keeps facing the swivel).
