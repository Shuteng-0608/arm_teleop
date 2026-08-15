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

After the commissioning checks above pass, run ROS-free scripted collection:

```bash
conda activate arm_teleop
python -m vptele.main_scripted \
  --config vptele/config/config_arm_right_moving_hole.yaml \
  --target-episodes 100 \
  --reject-action quarantine
```

Add `--show-ui` for the passive MuJoCo viewer and camera windows. Without that
flag the batch remains headless. `main_scripted.py` enables the scripted
controller only in memory; the checked-in interactive profile remains safely
disabled and no ROS or Vision Pro interface is initialized.
