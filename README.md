# Pangu Arm Teleopration System via Apple Vision Pro  

右臂 R30 / 7790 对比实验：见 [本次部署与运行说明](experiments/r30_pose_7790/README_ZH.md)。
`playback_right_teleop.py` 已默认使用随仓库提供的新 CSV 及配套初始关节角、臂角；
不改变下述 `main_ros.py` 原工作流。

1. Ensure ROS Master is running
```bash
roscore
```
2. Launch the arm inverse kinematics service:
```bash
rosrun arm_teleop ik_service_node
```
3. Launch the arm teleopration node:
```bash
rosrun arm_teleop main_ros.py
```
