# Plan: DXF parts tray → slots, calibration, picking, dashboard

Status (2026-11-07): **tray core + dashboard calibration built** on branch `tray-core` — DXF layout, config,
calibration math, hand-guided `tray-calibrate` + dashboard Calibration tab (corners mode default), pocket inventory,
tray API (`/api/tray*`), go-to preview, tests in `packing/tests/test_tray.py` / `test_arm_view.py`.
**Calibrated on the real arm** 2026-11-07 (plate corners, 2.15 mm RMS, tray tilted 69° from flat; user checked a
go-to visually). Kinematics fixed the same day: URDF zero = `calibrate center` pose (see CLAUDE.md). The
calibration was saved before the P=31 / feed-forward tuning came in from `main`. It's a static, hand-guided fit, so
it should still hold, but re-check with a go-to and redo it if a corner looks off.
**Not yet:** picking from pockets (`Cell.pick_tray`, per-type grip width), box handling + drop spot, optional
table-touch joint-offset calibration (only if tip-on-table readings drift beyond ~±5 mm).

As built (differs from the design below): DXF read with **ezdxf** (loops via `ezdxf.edgeminer`, polylines
exploded), rigid fit with **scipy** `Rotation.align_vectors`; fit rejected above **3 mm RMS**
(one point 10 mm off of 4 gives ~3.3 mm); config keys `size`/`spacing`/`grip_axis` per type, no
`grip_width` yet; `calibration_mode: corners|pockets` (corners = middle of each R12 plate corner, ids
`corner_bl/tl/br/tr`) + `reference_points` (pockets mode) in config; sim uses `sim_tray_cal.json`. Code: `packing/tray/`
(`dxf.py`, `layout.py`, `calibration.py`), `packing/tools/tray_calibrate.py`, CLI `tray` / `tray-calibrate`.

## Goal
Replace the 4 placeholder `part_slots` with a tilted, clamped parts tray whose pocket layout comes
from a DXF (`part.dxf`). The robot picks one item per order from a full pocket, marks it empty, and
never picks from it again until a human marks it restocked on the dashboard.

## Tray geometry (from `part.dxf`, units mm, origin = center of AA hole 1)
No text or layers in the file (all entities on layer `VISIBLE`). Pockets are found by shape:

| Type (sku) | Count | DXF feature | Pick point | Pitch |
|---|---|---|---|---|
| `aa` AA battery | 7 | circle Ø15.5 | circle center, x=0 | 17.78 |
| `9v` 9V battery | 5 | rect 27.5×18.5 | rect center, x=50.8 | 25.4 |
| `piezo` piezo speaker (Ø12) | 8 | pair of Ø1.3 circles 8 mm apart | midpoint of pair, x=88.9 | 15.24 |
| `glue` glue stick | 8 | circle Ø12.5 | circle center, x=114.3 | 15.24 |
| `breadboard` | 5 | slot 52×2.5 | slot center, x=158.75 | 25.4 |

Plate 204.3×132.1 mm, R12 corners. y runs 0…106.68 (AA/piezo/glue) or 0…101.6 (9V/breadboard).
Pocket IDs: `<sku>_<n>`, n=1 at y=0.

Every item sticks out **10 mm** above the tray face. Extraction: pull straight out along the tray
normal for **≥120 mm** (glue stick length, the longest) before any other motion.

## Design

### 1. `packing/tray/dxf.py` — tiny DXF reader (no ezdxf in env; don't add a dep)
Parse ENTITIES group codes (LINE, CIRCLE, ARC). Classify features by rule in config
(circle radius ranges, closed 4-line rects by size, pin pairs = two equal small circles at a given
spacing). Output `[Pocket(id, sku, x_mm, y_mm, yaw_deg)]`. `yaw` = pocket long-axis angle in the tray
plane (needed to set gripper roll for 9V and breadboard).

### 2. Config — `tray:` section in `cell.yaml` (replaces `part_slots`)
```yaml
tray:
  dxf: ../part.dxf
  calibration_file: data/tray_cal.json
  protrusion_mm: 10
  extract_mm: 120
  approach_mm: 30          # stand-off above the item top before descending
  types:                   # sku -> how to find it in the DXF + how to grip it
    # grip: fingers close along tray x (left-right) for all but breadboard (along tray y)
    aa:         {feature: circle, diameter: 15.5, name: AA battery,  grip_width: 14.5}
    9v:         {feature: rect, size: [27.5, 18.5], name: 9V battery, grip_width: 26.5}
    piezo:      {feature: pin_pair, pin_diameter: 1.3, spacing: 8, name: piezo speaker, grip_width: 12}
    glue:       {feature: circle, diameter: 12.5, name: glue stick, grip_width: 12}
    breadboard: {feature: slot, size: [52, 2.5], name: breadboard, grip_width: 2.5}
```
`catalog` derives from `types[*].name`. Per-type `grip_width` replaces the single `min_part_width`
for tray picks (breadboard 2.5 mm < current 3 mm threshold).

### 3. Calibration — `python -m packing tray-calibrate`
- User hand-guides the gripper tip (fingers closed = pointer) to N ≥ 3 reference points. Torque off
  → warn the user to hold the arm first. Joint reads only, no motion.
- Default reference points: pocket centers at the four extremes — `aa_1` (0,0), `aa_7` (0,106.68),
  `breadboard_1` (158.75,0), `breadboard_5` (158.75,101.6). (Plate corners are rounded, so the
  geometric corner is in mid-air; pocket centers are precise.)
- Tip position per point via `ArmModel.tip()` (FK on `gripper_frame_link`).
- Fit rigid transform tray→robot (Kabsch / SVD, DXF points at z=0) → `R, t`. Report per-point
  residual; refuse to save if RMS > 3 mm (as built). Tray normal = `R[:,2]` (sign chosen to point away from
  the table).
- Sanity checks (warn, not enforce): tray center azimuth plane contains the normal ("faces the
  swivel"); tilt angle reported.
- Save `data/tray_cal.json` (+ timestamp). Re-run whenever the tray is re-clamped.

### 4. Picking — `Cell.pick_tray(pocket)`
Robot-frame pocket point `p = R @ [x,y,0] + t`, normal `n`.
1. Joint move to `p + n*(protrusion + approach)` with gripper open, tool axis ≈ −n, roll from pocket yaw.
2. Straight line (IK, pre-validated) to `p + n*(protrusion/2)`; close; stall-detect grip vs `grip_width`.
3. Straight line out to `p + n*(protrusion + extract_mm)`.
4. Travel → place in box (existing pack flow, one `box_slot`).
Whole path planned and validated (reach, limits, jumps) before moving — same as `goto --line`.

### 5. Inventory
Pockets become `parts` rows (location = pocket id, sku = type). Picking = existing `pick_part`.
Pocket choice: lowest-numbered full pocket of the requested sku. Human restock is trusted.
New `Inventory.set_pocket(location, full: bool)` (event-logged) for dashboard toggles.

### 6. Dashboard + API
- `GET /api/tray` → geometry (outline, features, pockets with state). `POST /api/tray/pocket`
  `{id, full}`; `POST /api/tray/fill {sku?}`; `POST /api/tray/empty {sku?}`.
- To-scale SVG of the tray drawn from the DXF features; pockets colored full/empty; click toggles;
  per-type "fill all"/counts. Live updates via existing SSE (`tray` event kind — avoid as CSS class).
- `Shelf.available()` counts full pockets by sku; ordering agent sells one item per order.

### 7. Tests (sim)
DXF parse → 33 pockets with exact centers; Kabsch fit on synthetic tilted/rotated tray (with noise)
recovers transform; pick path validation with FakeArm; inventory toggle + pick-marks-empty;
API endpoints on spare port with temp cell.yaml copy.

## Known risk: 5-DOF approach alignment
The SO-101 tool axis always lies in the vertical plane through the base axis and the tip
(no independent yaw). Pockets off the tray centerline need the tool axis parallel to the tray
normal, which is *not* in that plane → unavoidable angular error ≈ atan(lateral offset / horizontal
distance from base). E.g. 80 mm off-center at 250 mm ≈ 18°. The tip path can still be exactly along
the normal, but the gripped item is held tilted by that angle relative to its hole — a 120 mm glue
stick in a Ø12.5 hole may bind. Mitigations: put the tray's short side (columns) across the arm's
view, keep the tray as far/narrow as reach allows, or accept and test. Planner will compute and
report the worst-case angle per pocket after calibration.

**Accepted by the user** (2026-11-07): tray will be placed far enough away; slight angle is OK.
Orientation confirmed: DXF x-axis runs left→right across the arm's view (AA column on the arm's
left, breadboards on the right), DXF y=0 row (bottom edge) nearest the arm, tray tilted back toward
the ceiling. Tray center x = 89.4 mm, so the piezo column is on the centerline. Angle per column vs
horizontal distance from the pan axis to the tray:

| column | lateral offset | 200 mm | 250 mm | 300 mm | 350 mm |
|---|---|---|---|---|---|
| aa | −89 mm | 24° | 20° | 17° | 14° |
| 9v | −39 mm | 11° | 9° | 7° | 6° |
| piezo | 0 | 0° | 0° | 0° | 0° |
| glue | +25 mm | 7° | 6° | 5° | 4° |
| breadboard | +69 mm | 19° | 16° | 13° | 11° |


## Decisions (2026-11-07)
- **Grip direction:** fingers close along tray x (left↔right as seen from the arm) for aa, 9v
  (across its 26.5 mm side — only ~7 mm gap between 9Vs vertically), piezo, glue. **Breadboard:**
  fingers close along tray y ("vertical grab", across the 2.5 mm thickness). Store as per-type
  `grip_axis: x|y` in the tray plane; wrist roll derived from it + calibration.
- **Calibration UX:** motors off, user hand-guides the closed gripper tip to each reference point
  and presses Enter; warn to hold the arm before torque off.
- **Destination:** item goes into a box (see below). A fixed drop spot will also be defined.

## TODO later (out of scope for the first build)
- **Box handling.** Box model in `box.step` (STEP AP242, Onshape; units meters). Bounding box:
  76.2 × 76.2 mm square (3 in), 77 mm tall, rounded corners, with a horizontal **grab tab** sticking
  out ~22.9 mm from one side at the top rim (y down to −60.96 mm). Arm grabs the tab to move boxes.
  Needs its own calibration (box supply / pack / output positions) — same hand-guide + Enter flow.
- **Drop spot** definition + calibration.

## Open questions
1. ~~Which tray direction faces the arm~~ — answered (see risk section).
2–4. Answered (see Decisions).
