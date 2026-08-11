# Pangu Arm Teleopration System via Apple Vision Pro  
1. Ensure ROS Master is running
```bash
roscore
```
2. Launch the arm inverse kinematics service:
```bash
rosrun arm_teleop ik_service_right_node
```
3. Launch the arm mujoco sim teleopration node:
```bash
rosrun arm_teleop main_mujoco.py
```

## Scripted dataset collection

Run automatic collection without ROS (headless by default):

```bash
python -m vptele.collect_mujoco --target-episodes 100
```

This path does not require `roscore`, ROS services, Vision Pro, or the ROS IK
node. Add `--show-ui` when a local MuJoCo viewer and camera windows are useful.
The existing ROS entry remains available for manual review and compatibility.

Keep the existing console review workflow:

```bash
rosrun arm_teleop main_mujoco.py --review-mode manual
rosservice call /scripted_insertion/run "{}"
```

Run a fully automatic batch that stops after retaining 100 accepted episodes:

```bash
rosrun arm_teleop main_mujoco.py \
  --review-mode auto \
  --target-episodes 100
```

Rejected episodes are moved to the sibling directory
`<hdf5_record_dir>_rejected` by default, so recursive dataset scans cannot
ingest them accidentally.
Every attempt is recorded in `<hdf5_record_dir>/batch_manifests/*.jsonl`.
Use `--reject-action delete` only after the quality thresholds have been
validated on representative data. `--max-attempts` provides an explicit
safety limit; when omitted, the collector allows five attempts per requested
retained episode.
The batch also stops after 10 consecutive rejections by default; this guard is
configurable through `scripted_controller.max_consecutive_rejections`.
