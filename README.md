# Pangu Arm Teleopration System via Apple Vision Pro  
```bash
CTRL + ALT + T # 打开终端
```
```bash
CTRL + SHIFT + O # 拆分终端
```

1. Ensure ROS env is activated
```bash
pangu # 每一个终端都要pangu一下
```
2. Launch the service:
```bash
roslaunch arm_teleop teleop_pure.launch # 上面的终端
```
3. Launch the arm teleopration node:
```bash
rosrun arm_teleop main_ros.py # 下面的终端，运行前需要工控机先 tele_run
```
