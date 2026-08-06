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

## Marvin M6 experimental entry

The Marvin model has a Python-only Vision Pro teleoperation entry. It does not
need `roscore`, `rosrun`, or the ROS `ArmIK` service:

```powershell
python vptele\main_mujoco_marvin.py --vp-ip <VISION_PRO_IP>
```

The entry automatically uses `ik_lib/libKine.dll` on Windows and
`ik_lib/libKine.so` on Linux, together with `ccs_m6_31.MvKDCfg`. Read
[`docs/marvin_mujoco_teleop.md`](docs/marvin_mujoco_teleop.md) before running.
