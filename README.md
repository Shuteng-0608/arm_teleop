# Pangu Arm Teleopration System via Apple Vision Pro  

## ROS-free MuJoCo environment

Create the project-specific Python 3.10 environment from the repository root:

```bash
conda env create -f environment.yml
conda activate arm_teleop
```

If the environment already exists, synchronize it with the checked-in file:

```bash
conda env update -n arm_teleop -f environment.yml --prune
```

This environment contains the MuJoCo simulator, Vision Pro stream client,
HDF5/image tooling, analysis libraries, and the ROS-free test dependencies. It
does not install ROS, `rospy`, catkin, or generated ROS message packages.

Load the moving-hole model in the viewer:

```bash
python scripts/view_mujoco_model.py \
  --model model/pangu_moving_hole_fixed_peg.xml
```

## ROS-free Vision Pro teleoperation

Start human teleoperation and manual HDF5 collection without ROS:

```bash
conda activate arm_teleop
python -m vptele.main_vr \
  --config vptele/config/config_arm_right_moving_hole.yaml \
  --vp-ip 172.20.10.2
```

Use `r` (or Enter) to reset/calibrate/start an episode, `k` to keep it, `d`
to discard it, `c` to recalibrate, and `q` to shut down safely. For hardware-
free commissioning, add `--synthetic`; add `--headless` to disable all local
windows. See [docs/vr_teleop_python.md](docs/vr_teleop_python.md).

## Legacy ROS teleoperation

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
python -m vptele.main_scripted --target-episodes 100
```

This path does not require `roscore`, ROS services, Vision Pro, or the ROS IK
node. Add `--show-ui` when a local MuJoCo viewer and camera windows are useful.
`python -m vptele.collect_mujoco ...` remains available as a compatibility
alias. The existing ROS entry remains available for manual review and
compatibility.

Run the moving-hole / fixed-peg scene explicitly:

```bash
python -m vptele.main_scripted \
  --config vptele/config/config_arm_right_moving_hole.yaml \
  --target-episodes 100 \
  --max-attempts 500 \
  --reject-action quarantine
```

The scripted entry forces automatic scripted mode only in its in-memory copy
of the configuration. It does not modify the YAML profile and never enables
ROS, Vision Pro, or ROS recording services.

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
