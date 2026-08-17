# Moving-hole / fixed-peg task

This scene reverses the original task roles while retaining the same circular
contact model:

- the robot carries a 24-segment circular blind-hole tool;
- a 45 mm long, 11 mm radius cylindrical peg is fixed in the world;
- the hole radius is 14 mm and its physical depth is 45 mm;
- the success target uses 43 mm of insertion travel, leaving 2 mm clearance
  from both the blind-hole bottom and the peg fixture base.

## Files

- Model: `model/pangu_moving_hole_fixed_peg.xml`
- Config: `vptele/config/config_arm_right_moving_hole.yaml`

The original `pangu_all_right.xml` and `config_arm_right_peg.yaml` remain
unchanged and continue to describe the moving-peg / fixed-hole task.

## Role mapping

| Role | MuJoCo object |
| --- | --- |
| Moving end-effector body | `hole_tool` |
| Moving control/success site | `hole_goal_site` |
| Fixed target body | `fixed_peg_fixture` |
| Fixed success site | `fixed_peg_tip_site` |
| First-contact target site | `fixed_peg_approach_goal_site` |
| Fixed peg collision geom | `fixed_cylindrical_peg` |
| Moving socket collision geoms | `tool_hole_ring_00` ... `23` |
| Force/torque sensors | `hole_ft_force`, `hole_ft_torque` |

## Commissioning sequence

The new config starts in manual Vision Pro mode and leaves scripted insertion
disabled.  Before enabling `scripted_controller.enabled`, verify:

1. the socket opening faces the fixed peg;
2. centered insertion is contact-free until the seated clearance;
3. an intentionally offset approach produces socket-rim contact;
4. the measured axial force sign agrees with the configured insertion axis;
5. gravity-compensated wrench is near zero away from contact;
6. task success triggers only near the seated pose.

View the model without starting teleoperation:

```bash
conda run -n arm_teleop python scripts/view_mujoco_model.py \
  --model model/pangu_moving_hole_fixed_peg.xml
```

Run the focused regression tests:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 conda run -n arm_teleop \
  python -m pytest -q test/test_moving_hole_model.py
```

After the commissioning checks above pass, run the 100-episode collision batch:

```bash
conda activate arm_teleop
python -m vptele.main_scripted \
  --config vptele/config/config_arm_right_moving_hole.yaml \
  --target-episodes 100 \
  --scenario collision \
  --reject-action quarantine
```

Run a separate 100-episode clean batch with:

```bash
python -m vptele.main_scripted \
  --config vptele/config/config_arm_right_moving_hole.yaml \
  --target-episodes 100 \
  --scenario clean \
  --reject-action quarantine
```

The clean scenario fixes the X/Z offset at zero, treats contact-free arrival at
the centered entrance as success, skips the post-contact retract, and disables
the in-hole disturbance. A debounced 3 N contact during the clean approach is
reported as `unexpected_contact` and the episode is rejected. Both scenarios
retain the same two-stage replay, sensor recording, success gates and terminal
hold.

Add `--show-ui` for the passive MuJoCo viewer and camera windows. Without that
flag the batch remains headless. `main_scripted.py` enables the scripted
controller only in memory; the checked-in interactive profile remains safely
disabled and no ROS or Vision Pro interface is initialized.

## Scripted rim-contact coverage and approach control

The active rim-contact phase uses measured `hole_goal_site` feedback rather
than advancing an open-loop Cartesian cursor. Free space, far approach, near
approach and the final probe have separate velocity limits. Contact is first
debounced at a low force and then allowed a bounded additional command travel
to build the sampled 8-12 N target; the 40 N overload guard remains separate.

Initial rim offsets are no longer fixed at 10 mm. One deterministic cycle
contains four radii (`4, 6, 8, 10 mm`) and 24 angular sectors, for 96 cells.
The configured seed reproducibly shuffles the cells and adds at most 5 degrees
of angular jitter. `error_coverage_start_cycle` and
`error_coverage_start_index` provide explicit resume control across separate
process launches.

Each two-stage episode records both planned coverage fields and measured
contact diagnostics, including:

- coverage cycle/index and radius/angle cell;
- target and actual contact offset;
- first-contact and complete-approach durations;
- contact peak force and bounded push depth;
- whether the sampled contact-force target was reached.

These values are present in the stage-2 episode context, the stage-1 HDF5
metadata and `stage1_summary.json`. The insertion itself remains the existing
centered waypoint path until the configured in-hole disturbance depth.

## In-hole disturbance and lateral recovery

The first recovery version keeps socket orientation fixed and only corrects
translation in the plane normal to the insertion axis (world X/Z in this
scene). One deterministic disturbance is injected per episode. A 48-cell
cycle covers three insertion depths (`30%, 55%, 80%`), eight lateral
directions and two commanded amplitudes (`3.4, 3.8 mm`). The configured cursor
allows a later process to resume the coverage cycle.

Force-controlled decisions use only
`get_gravity_compensated_ft_wrench_world()`. This path resolves the site from
the configured `hole_ft_force` sensor, subtracts the predicted wrench of the
`hole_tool`, rotates the result into world coordinates and has no raw-force
fallback. Configuration validation also rejects in-hole correction unless the
dataset compensation mode is `gravity` and tool bodies are configured.

Gravity compensation does not remove inertial wrench during motion. To avoid
classifying the smooth disturbance ramp as contact, force detection is gated
until at least 70% of the disturbance has been commanded. A debounced 2 N
lateral threshold confirms contact; the bounded ramp may then build a 5 N
recovery condition. Axial insertion pauses while a filtered, speed- and
travel-limited lateral admittance command releases contact. A 25 N local
limit and the existing 40 N global overload protection abort unsafe episodes.

The collection stores the executed trajectory without per-action phase labels,
expert-action flags, or training-loss masks. Disturbance and recovery remain
part of the same unannotated command sequence.

## Success and terminal hold

Success requires the seating-site distance to remain below 1 mm while the
gravity-compensated force stays below 5 N total and 3 N laterally, and the arm
joint-speed norm stays below 0.10 rad/s. This prevents a moving or wall-loaded
socket from being accepted solely because its virtual sites briefly overlap.

Terminal hold uses a position-actuator target with inverse-dynamics gravity
feed-forward. Stage-2 replay stops before writing another trace command as
soon as task success is latched, so post-success retract commands from stage 1
cannot override the hold. The hold completes after at least 0.5 s and 0.25 s
of continuous stability, with a 2 s maximum. A compensated force above 8 N
total or 5 N laterally for 0.05 s re-anchors the controller at the actual pose
to release servo preload, records `terminal_hold_safety_abort`, and rejects the
episode.
