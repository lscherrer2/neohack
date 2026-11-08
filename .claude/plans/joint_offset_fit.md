# Joint zero-offset fit from desk touches

Written 2026-11-07. Follows `FLOOR_CALIBRATION.md`.

## Why
After a 7-point desk calibration the user saw the touched points (motors off, so no sag) sit **right on the desk
near the arm and higher and higher further out**, the same left and right. A plane can't fit that (it's
rotationally symmetric about the swivel). It is the signature of a small zero error on `shoulder_lift` /
`elbow_flex`: the kinematics zero is the eyeballed calibration pose (`CLAUDE.md`: "a table-touch joint-offset fit
is the planned fix").

## Model
- `ArmModel(ranges, offsets_deg={joint: deg})`: the offset is added to that joint's URDF angle (`zero_deg`), so
  FK **and** IK both use it. Taught poses / servo units / the servo calibration don't change.
- Saved per arm: `calibration/<robot_id>_joint_offsets.json` (`kinematics.offsets_file` in `cell.yaml`); sim
  servers use `data/sim_joint_offsets.json`. `kinematics.arm_model(robot_id, offsets_file)` builds the model
  everywhere (shop hub, CLI, remote).

## Fit (`packing/motion/joint_offsets.py`)
- Each desk touch now stores its joint pose too (`DeskPlane.poses`, in-memory points).
- Unknowns: Δlift, Δelbow (deg) + desk plane a, b, c. Residual = jaw-tip z − plane(x, y) (mm), jaw tip = FK with
  the offsets. Weak priors keep it stable: offsets σ 3°, desk tilt σ 1°. `scipy.optimize.least_squares`.
- Needs ≥ 6 points, spanning ≥ 8 cm in distance from the swivel and ≥ 40° around it. The result is flagged not
  applicable if an offset's std > 0.5°, |Δ| > 5° or it doesn't improve the RMS.
- Wrist offset isn't fitted: with the gripper pointing down it barely changes the tip height.

## Apply (dashboard Desk panel → Fit joint angles → Apply)
- Saves total offsets = current + Δ (the fit runs on top of the current model).
- Reloads the hub model; re-derives the desk points / saved plane and every box (from its capture joints).
- The **tray calibration must be redone** (it stores positions only).
- Remove = delete the file, same re-derivation.

## Status (2026-11-07): implemented, sim-tested, not run on hardware
- Synthetic check (true arm lift +1.5°, elbow −1.0°, flat desk): 25 touches recover +1.47° / −0.96°
  (flat to 0.02 mm); 12 touches can't separate lift from elbow to ±0.5° (correlation ~0.84 through heights
  alone) → falls back to lift alone (+1.2°), 1.2 → 0.26 mm RMS. Tests: `packing/tests/test_joint_offsets.py`.
- `gravity_feedforward` still builds its own model without offsets (1–2° barely changes gravity torques).
- Next on hardware: capture ~15–25 touches (near/far × left/middle/right), Fit, check the per-point table goes flat,
  Apply, re-calibrate the tray, re-check box heights left → middle → right.
