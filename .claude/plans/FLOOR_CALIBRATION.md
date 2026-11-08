# Floor (desk) calibration — implementation handoff

"Floor" = the desk / table surface the arm, the boxes and the parts tray stand on. Written 2026-11-07 from the
discussion with the user. **A first version is already implemented** (called "desk" in the code, commit `aa9dd1c4`);
this document says what floor calibration should do, what exists, and what is still open.

## 1. Why it's needed (what we saw)

- Box moves came in **too low on one side** (approaching the left box spot) but at the right height on the way back
  (right/middle → left). The planner was checked: the planned tool heights are **identical both ways to within
  0.05 mm** (approach 74.97 mm, grip 75.02 mm, set-down 74.98 mm in both directions). So the difference is physical,
  not in the plan.
- The planner assumes **"up" = the arm base's z axis and the desk is flat at one height everywhere**. If the base sits
  slightly tilted, or the desk isn't flat, the real desk height relative to the arm changes from spot to spot:
  **1° of tilt over ~30 cm ≈ 5 mm**. That matches the symptom.
- Heights of box spots other than the calibrated one were taken from the box's own calibration: the pack spot (middle,
  `boxes.pack_spot_mm` = (355, 0) mm) and the output spot (the supply spot mirrored to the right) reuse the supply
  box's bottom height. Those are the spots a tilt makes wrong.
- Smaller contributors (not fixed by floor calibration): servo sag under load (gravity feed-forward models the arm,
  not the box), and the arm's control getting less precise when reaching far out. The user accepts a few mm of play.

## 2. What floor calibration should do

### Capture
1. Motors off (Controls → Motors → Released, or during calibration); a person holds the arm.
2. Touch the **closed jaw tip** to the desk at **3 or more spots far apart** — left, middle, right, ideally near where
   boxes go (supply spots on the left, pack spot in the middle, output spots on the right).
3. Each capture reads the tool point (FK of the averaged joint reading, `ArmHub.read_tip`).

### Fit + checks
- Least-squares plane **z = a·x + b·y + c** (robot base frame, meters) through the points.
- Refuse (message shown to the user) if:
  - fewer than 3 points;
  - the points are bunched together or in a line (enclosed area < 40 cm², `MIN_AREA_M2`);
  - they don't lie on a plane (RMS > 3 mm, `MAX_RMS_MM`) — someone pressed into the desk or touched the tray.
- Show the live fit (RMS, tilt in degrees) before saving; save on request.
- Saved separately for sim runs (never overwrite the real calibration from a sim server).

### Use
- Every box spot that isn't the box's own calibrated spot takes its height from the plane, **relative to the
  calibrated spot**:

  `bottom(at xy) = bottom(calibrated) + plane(xy) − plane(calibrated xy)`

  Only height *differences* are used, so a constant offset cancels — in particular the jaw tips stick out ~6 mm past
  the tool point (`gripper_frame_link`), which would otherwise bias every touched point.
- The calibrated (supply) spot keeps its exact calibrated grip (it replays the hand-guided pose).
- Without a floor calibration everything behaves as before (flat desk at the calibrated box's height).

## 3. What exists today (commit `aa9dd1c4`)

| Piece | Where |
|---|---|
| Plane fit, checks, save/load, `height_at` | `packing/boxes/desk.py` (`DeskPlane`, `fit_plane`, `DeskError`) |
| Config | `cell.yaml` → `desk: calibration_file: data/desk_cal.json`; `CellConfig.desk_file`, `CellConfig.desk_plane()` (re-read every call) |
| Sim file | `packing/shop/__main__.py` `make_hub`: sim servers use `data/sim_desk_cal.json` |
| Used by | `packing/boxes/move.py` `box_pose()` — pack and output spots follow the plane |
| API | `GET /api/desk` (points, live fit or fit error, saved plane) · `POST /api/desk/capture` (409 if motors on) · `DELETE /api/desk/points` / `DELETE /api/desk/points/{i}` · `POST /api/desk/save` (422 with the reason if the fit is refused) · `DELETE /api/desk` |
| UI | Calibration tab → **Desk** section (`web/src/calibration/DeskBox.tsx`): instructions, Capture point N, point list with ×, live fit, Save desk, Clear points, Remove calibration. Touched/saved points drawn as small discs in the 3D view (`Scene.tsx`, `deskPoints`) |
| Tests | `packing/tests/test_desk.py`: tilt recovery, refusals (too few / collinear / bunched / bumpy), relative heights, box spots following a tilted desk, API capture → fit → save → remove |

Touched points are kept **in server memory** until saved (a restart drops unsaved points — same as the tray session).

## 4. Still open (suggested next steps, in order)

1. **Hardware run**: calibrate the floor (4–6 points spanning left/middle/right), check the reported tilt, then repeat a
   left → middle → left box move and compare the approach heights. If the tilt is ~0 and the heights still differ,
   the cause is sag / reach precision, not the floor (see 5).
2. **Use the plane for the box capture check**: `boxes/geometry.box_from_grasp` refuses a capture if the box bottom
   is > 20 mm from z = 0. It should compare with `plane(xy)` (minus the jaw-tip offset, or relative to other boxes).
3. **Use the plane in the tray "stand" obstacle**: `geometry.tray_blocks` extends the tray's stand block down to
   z = 0; it should go down to the floor plane under the tray.
4. **Arm/box vs floor collision**: the 3D checks (`move.check_path`, `check_arm_clear`) know boxes and the tray but not
   the desk. Add "no arm point / carried box corner below `plane(xy)` + margin" (the carry already never dips below its
   lifted height, but free moves and picks aren't checked against the desk).
5. **If sag turns out to be the cause** (heights still off with a flat floor): add the box's weight to the gravity
   feed-forward while holding a box (`motion/kinematics.gravity_feedforward` — URDF masses only today), or a
   configurable `boxes` height offset for the grip/set-down (the user accepts a few mm of play).
6. Other z = 0 assumptions worth moving to the plane: `remote.min_z_m` (LLM contingency safe box), the pick's lift
   checks, the `CLAUDE.md` note "jaw resting on the table reads z ≈ +5…+10 mm".
7. Optional UI: draw the fitted plane (a translucent patch) and show per-spot height corrections (e.g. "pack spot
   −3.2 mm, box 1 output +1.1 mm") in the Boxes list.
8. Optional: tilt "straight up/down" box lifts along the plane normal instead of the base z — only matters for large
   tilts (> ~2°).

## 5. Acceptance (hardware)

- A 4–6 point floor calibration saves with RMS ≤ 1.5 mm.
- With it saved, a box moved left → middle → right → left lands within ~2–3 mm of the same height each time (the box
  sits flat; no edge touches first), and the approach no longer comes in low on the left.
- Removing the calibration restores the old behavior exactly.

## 6. Related

- `.claude/plans/boxes_and_reservations.md` — boxes, grip, carry strategy, all the changes that led here.
- `CLAUDE.md` — "Desk plane" bullet under Shop.
