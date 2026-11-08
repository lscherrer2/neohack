# Reach compensation (aim higher further out)

Written 2026-11-07. Follows `joint_offset_fit.md`.

## Why
After the desk plane and the joint-offset fit the user said the joint fit "didn't do anything" and asked for an
optional empirical correction: the further out the aim point, the higher it goes — ½ in at 70 % of max distance,
"increased sag correction basically".

## What
- `lift(r) = lift_mm · (r / (at_reach · max_reach))^power`; r = tool point's horizontal distance from the swivel;
  `max_reach_m(model)` = farthest tool-point r over the joint ranges (grid, cached; ≈ 441 mm).
- Defaults `lift_mm` 6 (was 12.7; user, 2026-11-07, after the averaged tray fit), `at_reach` 0.7, `power` 2 (sag grows with the lever arm; ~0 close in where touches read
  right). Power 2 is our pick — the user only gave the 70 % point; set `power: 1` in `cell.yaml` `reach_comp:` for
  a straight line from the swivel.
- Run time only: `raise_steps(model, steps, rc)` re-solves every JointMove pose / LineMove path pose (and Grip
  `on_fail` steps) with the target z raised, orientation weight 0.5; > 2 mm IK error or a joint over ±100 →
  ValueError before anything moves. Called in `shop/goto_runner.py` (go-to, pick, box move) and
  `boxes/packer.py` (orders; a failure there = `NotStarted`, box given back).
- Not applied to: plans / their collision + desk checks / dry-run paths (true aim), hand-guided calibrations,
  teach / CLI moves, holds.
- UI: Calibration tab → Reach compensation: checkbox, mm-at-70 % field, table of lift at 30/50/70/90 %.
- Tests: `packing/tests/test_reach_comp.py`.

## Caveats / next
- Touch-calibrated spots far out get the lift too: box supply spots (~32 cm) aim ~14 mm higher and the tray
  (~37 cm) ~18 mm higher than calibrated. If the error really is servo sag that's the point; if picks start
  missing high, lower `lift_mm` or turn it off.
- Not run on hardware yet. Tune `lift_mm` from a box move left → middle → right.
