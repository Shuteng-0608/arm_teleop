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
