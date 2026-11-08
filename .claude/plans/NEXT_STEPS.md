# Next steps: one order procedure (TrayPacker), 2026-11-07

**Goal:** one procedure for a whole order, run by `TrayPacker` (`packing/boxes/packer.py`, program from
`packing/boxes/pack.py`):

1. take an empty box from its supply spot (left),
2. move it to the pack spot (middle, `boxes.pack_spot_mm` = (355, 0) mm),
3. pick each ordered part from its reserved tray pocket and drop it into the box,
4. move the packed box to its output spot (right) for pickup.

Most of this already plans and runs in the simulator (see `boxes_and_reservations.md`). The work below is
cleanup, a new release pose, inventory bookkeeping, and the first hardware runs.

## Decisions (user, 2026-11-07)
- **Demo quality, not production.** Assume moves succeed. No resume, no retries, no handling of bad states.
- **TrayPacker is the only packer.** Delete the old `Cell` path.
- **Release above the rim is fine** (a drop, not a gentle place). Turn the jaws to the side for the release
  (see step 3).
- **Boxes only move left → right.** Supply (left) → pack (middle) → output (right, for pickup). They never
  come back. New boxes come in only through Calibration → "Add box".
- **Only failure case to handle:** the gripper closes on nothing → mark that pocket empty and stop the order.

---

## 1. Delete the old Cell path
Files that use it: `packing/cell/procedures.py`, `packing/service/worker.py` (`RobotPacker`),
`packing/service/__main__.py`, `packing/shop/__main__.py` (`make_packer`), `packing/cli.py`,
`packing/tests/test_packing.py`.
- [ ] Delete `cell/procedures.py` and `RobotPacker`. `make_packer` with `--robot` → always `TrayPacker` (it
      needs the hub, so only in-process: `start --robot` / `serve --robot`). `robot --robot` as a separate
      process goes away; `robot` without `--robot` stays as the sim packer.
- [ ] CLI: remove `pack`, `move-box`, `set-box`, `clear-output`, `teach-all`/`check` (if they only exist for
      Cell poses). Keep `teach`/`goto`/`where`/`home`/`release`/`tune`/`demo`/`tray`/`tray-calibrate`/`status`/
      `events`/`set-part`/`restock`.
- [ ] `cell/inventory.py` stays (the shelf uses it for tray pockets). Drop its box part (box table/`restock` of
      "box supply spots"): boxes live only in the orders DB `boxes` table.
- [ ] `cell.yaml`: remove the Cell-only placeholders (`box_in_*`, `pack_1`, `box_out_*`, `box_slot_1`,
      `box_slots`, `pack_station`, `part_slots`, if nothing else reads them). Check `config.py` and
      `max_items_per_order` (which uses `box_slots` today).
- [ ] Parked `packing/service/`: keep `orders.py` and `vapi.py` (the shop uses them). Delete or stub anything that
      only fed `RobotPacker`.
- [ ] Tests: drop or port `test_packing.py`. The whole suite passes.

## 2. One-way box flow
- [ ] Box moves only go forward: supply → pack → output. Remove `to: supply` from `/api/arm/box-move` and the
      dashboard Move-a-box control. Remove `POST /api/boxes/{id}/reset` (or keep it only as a manual "this box
      is empty at supply again" fix; decide while doing it).
- [ ] **Pack spot taken:** if another box is already at `pack`, refuse to start an order with a plain message
      ("box N is still at the pack spot"), not a collision error.
- [ ] **Output spot:** each box's output spot = its supply spot mirrored right (one per box, so finished boxes
      never land on each other). Keep that. Packed boxes left at output stay obstacles for later moves (they are
      already).
- [ ] **Picked up:** a "Picked up" button on a packed box (Orders or Calibration tab) deletes the box record, so
      it stops being an obstacle (`DELETE /api/boxes/{id}` already exists; just label and place it).
- [ ] No empty box at supply → the order isn't started; message "add a box" (exists: `BoxError`).

## 3. Release with the jaws turned to the side
Today `plan_drop` points the gripper straight down over the box. That's only reachable up to ~10 cm above the
table, which limits the pack spot and the drops (365 mm+ already fails).
- [ ] New release pose: the tool does **not** have to point down. Roll the wrist so the jaws close
      **horizontally** (gripper x level), so when they open there is nothing under the part and it falls straight
      down. Tool level or tilted down, whichever reaches; prefer level/slightly down for range.
- [ ] Place the part's center (jaw point + half its length along the tool axis) over the box opening, jaws/part
      above the rim + `RIM_CLEAR_MM`. Long parts (glue ~120 mm) stick out past the 76 mm opening; for a demo, the
      center over the opening is enough (it tips in).
- [ ] The wrist roll may have to turn ~90° from the pick roll while holding the part. `wrist_roll` range is
      ~337° (raw 142–3972), so it fits; plan the roll during the free move to the box.
- [ ] Keep `check_clear_of_box` (the part/arm must not hit the box on the way in).
- [ ] Sim check: all 33 pockets plan at pack spot 355. Then see whether the pack spot can move (more room).
- [ ] Tests in `packing/tests/` (jaw axis level at the release, part center over the opening, above the rim).

## 4. Mark parts removed as they're taken
Today `shelf.consume_slots(order.slots)` runs once, after the whole order.
- [ ] After each successful grip (the step after `check_holding` / pull-out): mark that pocket's part used
      (`consume_slots([slot])`), so the tray map and stock update live. Drive it from `run(..., on_step=...)`
      by step label/index (planner returns which step finishes item i).
- [ ] Order → `packed` and box → `packed` at `output` at the end (already).
- [ ] Event-log line per item ("item 2/3: 9V from 9v_1 → box 1").

## 5. Empty pocket = mark empty and stop
- [ ] If the grip closes on nothing (`check_holding` fails; the plan's `on_fail` already opens and backs out):
      mark the pocket **empty** (`shelf.set_slot(slot, False)`; the reservation then moves or goes short by
      itself), log it, stop the order → `failed`. Whatever already happened (box at the pack spot, parts in it)
      stays as is; a person sorts it out.
- [ ] Test with `FakeArm` without `gripper_block` (missed grip → pocket empty, order failed, no crash).

## 6. Hardware bring-up (supervised, user says "go" before each motion)
- [ ] Tray on the LEFT, recalibrate (Calibration tab, corners).
- [ ] Add the boxes (supply on the left, centers ≥ 30.5 cm from the swivel, ≥ 12 cm apart).
- [ ] Measure part lengths (`length_mm` per tray type; estimates today) and fix `cell.yaml`.
- [ ] Dry run: Orders → Check on a 1-item order; look at the planned path.
- [ ] Piece by piece on the real arm: box move supply → pack; one pick (Controls → Pick…); one release over the
      box; box move pack → output.
- [ ] One full 1-item order via Action, then a 3-item order.
- [ ] Mind the monitor near the arm: watch the first runs of every new move.

## 7. Docs
- [ ] Update `CLAUDE.md` (layout, CLI, shop/boxes sections), `ROADMAP.md` (layers 5–7),
      `boxes_and_reservations.md` (side release, one-way flow), `packing/README.md`.

## Not doing (demo)
Resume after a crash, retries / other units after a miss, dropped-part detection, pause/continue on Stop,
volume check for 10-item orders, customer notification (later, with Vapi).
