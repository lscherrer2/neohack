# Tray pick procedure (2026-11-07)

Goal: pick any part type out of its tray pocket — approach along the tray normal, grip ~0.15 in off the face,
pull straight out (≥ 12 cm: the longest part; 15 cm preferred), lift, hold. Drop spot: **TBD, not implemented**.
Everything (pocket position, normal, grip direction) comes from the tray calibration.

## Motion programs — `packing/motion/program.py`
G-code-like "nouns + verbs", but **no modal state** (each step says absolute vs relative itself).
- Plan time: `ProgramBuilder(model, start)` verbs `joint_to(T)`, `line_to(p)`, `line_by(d)`, `open_gripper()`,
  `grip(min_width, on_fail=[...])`, `check_holding(min_width)`, `mark()/reset()` for trying alternatives. Each verb
  is turned into a concrete step (IK solved, reach / joint limits / jumps validated) → a program that builds is
  fully checked before anything moves.
- Steps: `JointMove`, `LineMove` (dense joint path + mm/s), `OpenGripper`, `Grip`, `CheckHolding`; `describe()`.
- Run time: `run(steps, motion)`. `on_fail` steps run only on `GraspError` (Stop / following error → no
  recovery motion).
- `GotoPlan.steps` uses it too; `shop/goto_runner.GotoRunner` runs any plan with `.label/.steps/.summary()`.

## Pick planner — `packing/tray/pick.py`
One procedure for every type; per-type differences are data (`tray.types[*].grip_axis`, `min_grip`).
Program: open → free move to approach point (`protrusion + approach_mm` out) → line in to `grasp_standoff_mm`
(3.81) off the face → grip (miss: open + back out) → line out `pull_mm` along the normal → check holding →
line up `lift_mm` → check holding → hold.
- Jaws close along gripper_frame **x** (checked from the URDF jaw joint); tool z = approach axis.
- IK: `AxisIK` (placo): tip position **hard**, tool z → (pitched) inward normal and tool x → grip direction
  soft (z weighted higher). Projecting into "the arm plane" by hand failed: the tool point sits 7.9 mm off the
  wrist-roll axis, so at roll ±90° the plane isn't through the tip (5–6 mm IK miss).
- **Pitch search**: the tray faces the arm, so a straight pull heads at the base and a level tool runs out of
  elbow travel (sim, real calibration: max level pull 40–100 mm). Tool pitched down lets the wrist take up the
  fold. Tries pitch 0, 10, … `max_pitch_deg` (40); smallest that fits `pull_mm` (150), else `extract_mm` (120).
  The two ± jaw directions (wrist roll 180° apart) are both tried; tie-break = roll nearest mid-range.
- Reported per plan: `pitch_deg`, `tilt_deg`, `pull_twist_deg` (5-DOF: tool yaw follows the pan while the tip
  moves toward the base off-center), `jaw_off_deg`.

### Real-calibration results (sim, 2026-11-07): all 33 pockets plan
| type | pitch | twist over pull | jaw off |
|---|---|---|---|
| aa | 20–40° | 8–12° | 13–16° |
| 9v | 40° | 4–6° | 6° |
| piezo | 30–40° | ~1° | ~1.5° |
| glue | 30–40° | 3–5° | 5–6° |
| breadboard | 20–40° | 7–10° | **21–41°** |

Twist grows with pull length; what matters is the twist while the part is still in its pocket (≈ part length).

### Gripper opening (2026-11-07)
Fully open (`gripper.open: 60`) is ~98 mm at the fingertips — far too wide. Picks open to the part width +
`gripper.open_margin_mm` (10): `GripperCfg.open_for(width)` = chord 2·L·sin(θ/2) inverted, L = `jaw_length_mm`
(80, URDF pivot→tool point estimate), θ per unit = calibrated gripper range / 100 (1.27°). Part width = type
`grip_mm` or the DXF feature (circle Ø, rect side along the grip axis); pin_pair types must give `grip_mm`
(piezo 12: estimate). Values: aa 14.5, 9v 21.4, piezo 12.5, glue 12.7, breadboard 7.1 units. The runner logs the
measured grip width (`last.grip_width`, activity "gripped at width …") → use it to check L / the widths.

### Fixed jaw offset + jaw tips (2026-11-07, from the SO-101 meshes)
Measured from TheRobotStudio/SO-ARM100 `wrist_roll_follower_so101_v1.stl` (fixed jaw) and `moving_jaw_so101_v1.stl`
in the tool frame: the **fixed jaw's inner face is at x ≈ 0 (through the tool point)**, its body on +x; the
moving jaw is on −x, opens toward −x, touches at URDF −10° (= gripper 0). So the planner aims the tool point
beside the part: `aim = pocket + side · (grip_mm/2 + open_margin_mm/2)`, side = tool +x in the tray plane
(`side_offset_mm`). Both jaw orientations (roll 180° apart) are planned; roll nearest mid-range wins.
Mesh check at the planned grasp (aa_1/3/7, glue_4, 9v_2, piezo_4): fixed jaw 4–5 mm clear of the part, open moving
jaw 6–7 mm, moving jaw meets the part at 8–18 units (≫ min_grip 3).
**Open: jaw tips stick out ~6 mm past the tool point along z.** At the planned grasp the fixed-jaw tip reaches
~2 mm *below* the calibrated face (pitched). The tray was calibrated by touching with the tips, so the calibrated
face is likely ~5–6 mm in front of the real plate → real tips ~4–5 mm off the plate, ~5 mm overlap of the 10 mm
protrusion. Proper fix (needs the user's OK): make the tool point the real fingertip (gripper_frame + ~6 mm z, or
a `fingertip` frame) and recalibrate the tray; then grasp_standoff = real tip clearance.

## Interfaces
- `POST /api/arm/pick {pocket, confirm_move}` — dry run unless `confirm_move`; same runner/Stop/state
  (`GET /api/arm/goto`) as go-to. Does **not** change inventory (test; drop spot TBD).
- Dashboard **Controls** tab → Procedures: pocket (grouped by type, full/empty shown) → **Pick…** → confirm dialog
  with the plan → run; Stop; live step + result (grip width). Motors Locked/Released on the same tab.
- Config (`cell.yaml` `tray:`): `grasp_standoff_mm`, `pull_mm`, `lift_mm`, `max_pitch_deg`, `pitch_step_deg`;
  per type `min_grip` (gripper units; breadboard 1 — tune).

## Open / next
1. Supervised real test, one pocket at a time, worst first (aa_2/aa_3 pitch 40; breadboard_1/2). Risks the sim
   can't see (no gripper meshes): pitched gripper hitting the row above; less finger overlap on the 10 mm
   protrusion when pitched; grip widths (`min_grip`) per type.
2. Breadboard: jaws tilt with the pitch (up/down grip lies in the pitch plane). If it slips: per-type
   `pull_mm` (its real depth) so it needs less pitch, or a per-type pitch cap.
3. Optional per-type `pull_mm` (part length) → less pitch and twist for short parts.
4. Drop spot / box place (TBD), then `Cell.pick_tray(sku)` with inventory transitions (pocket → in_gripper →
   box) and `RobotPacker` using it.

### Change 2026-11-07 (later): per-part pull-out, 4 mm tolerance
"no tool pitch up to 40° fits a 120 mm pull-out from 9v_…": every part had to pull 120 mm (the glue stick's need).
Now per part: needed = (length_mm − protrusion) + 10 mm (it can't sit deeper than that), preferred = needed + 40,
max `pull_mm` 150: AA 50/90, 9V 48/88, piezo 15/55, glue 120/150, breadboard 40/80 (min/preferred). `extract_mm`
is no longer used. Planning tolerance 4 mm / 6-unit line steps (user: a few mm of play). Sim, tray on the left:
33/33 at 33 cm out; at 28–30 cm the lowest 3–4 glue-stick rows can't make their 120 mm pull (reach).

