# ROS-free Vision Pro teleoperation

`vptele.main_vr` provides the human-operated collection path without importing
`rospy`, generated ROS services, or the C++ IK node. It reuses the same MuJoCo
controller, contact dynamics, force safeguards, CCTV force HUD, task-success
auto-stop, target scheduler, and HDF5 recorder as the legacy entry.

## Setup

```bash
conda env update -n arm_teleop -f environment.yml --prune
conda activate arm_teleop
```

The environment pins `avp-stream==2.51`. Both its `TrackingData` object and the
legacy dictionary representation are accepted. A right wrist must resolve to
a finite 4x4 transform; legacy frames may provide shape `(1, 4, 4)`.

## Real Vision Pro

Start the Tracking Streamer app, put the headset and workstation on the same
network, then run:

```bash
python -m vptele.main_vr \
  --config vptele/config/config_arm_right_moving_hole.yaml \
  --vp-ip <vision-pro-ip>
```

The first valid wrist pose establishes a safe stationary reference. Starting
each episode resets the arm and recalibrates that reference before accepting
commands. The legacy axis mapping is preserved:

| Vision Pro wrist offset | MuJoCo tool offset |
| --- | --- |
| `+Y` | `+X` |
| `+Z` | `+Y` (0.8 scale) |
| `+X` | `+Z` (0.8 scale) |

Tool orientation is held at the calibrated MuJoCo orientation, matching the
old teleoperator. IK uses MuJoCo forward kinematics and `mj_jacSite` in a
private `MjData`, with damped least squares, joint-limit clamping, per-frame
Cartesian step limiting, workspace offset limits, and actuator gravity
feed-forward.

Commands at the `vr-teleop>` prompt:

- `r` or Enter: reset, recalibrate, start HDF5 recording and control.
- `k`: stop and keep the active or auto-completed episode.
- `d`: stop, discard the episode directory, and retry the target.
- `c`: recalibrate the wrist reference.
- `q`: stop threads, flush an active recording as interrupted, and exit.

The final CCTV frame sent to Vision Pro is the same rendered frame that carries
the force HUD. Raw HDF5 camera frames remain free of operator overlays.

## Hardware-free verification

Run the whole entry with deterministic official-shape tracking frames:

```bash
python -m vptele.main_vr --synthetic
```

This validates initialization, calibration, IK, contact simulation, manual
recording, keep/discard, and shutdown. It does not validate headset networking
or WebRTC video return. For CI/headless use:

```bash
python -m vptele.main_vr --synthetic --headless
python -m pytest -q
```
