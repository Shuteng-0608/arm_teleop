# R80 / 300413 实验部署说明

本次将右臂在线播放器默认实验改为 R80 / 300413。工程 CSV 和配套初始关节、
臂角一起提交到 `arm_teleop` 的 `for_cjp` 分支，不再依赖运动学仓库中的结果目录。
旧 R30 / 7790 文件和常量保留作为历史记录。

## 改动范围

- `vptele/core/right_arm_trajectory.py`：新增 `R80_CLEARANCE_INITIAL_RIGHT_JOINTS`
  和 `R80_CLEARANCE_INITIAL_RIGHT_ARM_ANGLE`，默认 CSV 指向本目录。
- `vptele/playback_right_teleop.py`：MoveJ、IK 历史及入口限位校验统一使用上述初值。
- 不修改 `main_ros.py`、坐标映射、IK 服务、冻结算法、限位、MoveJ/tele 顺序、
  左臂反馈保持或双臂消息格式；下位机无需因本次换姿态修改代码。

## 输入与初值

- CSV：`R80_pose_300413_engineering.csv`，1186 帧，30 Hz，39.5 秒。
- SHA-256：`40c71bce5f56c634bbc52c1e553b60dcd919a865e980d5cc288277d5c0e79e96`。
- 时序：0–15 秒保持；15–37.5 秒径向进入、绕圆、径向返回；最后保持 2 秒。
- 半径：80 mm（8 cm）。原 VP 坐标 XZ 圆按既有映射成为机器人 YZ 平面圆；
  圆心 `[0.3011,-0.358,0.2282]` m 不变。固定姿态相对 7790 改变约 52°。
- 生成器：独立 `trajectory_generator` 仓库的 `scripts/generate_engineering_csv.py`。
  本目录保存生成配置 `trajectory.yaml`、输入 `first_frame_template.csv` 和结果 CSV；
  CSV 为已经数值验证的生成物逐字节副本，没有重采样或手工改列。

```python
# Offset 执行关节 q1～q7，rad
R80_CLEARANCE_INITIAL_RIGHT_JOINTS = [
    0.0373064520992315, -0.4888033056593821, -0.3525297378433601,
    1.4714625102873358, -0.0006633785666672, -0.6438443410081743,
    0.1368475907974190,
]
# Standard 选择器臂角，rad
R80_CLEARANCE_INITIAL_RIGHT_ARM_ANGLE = -0.8290313946973062
```

详细初值和来源见 [pose.json](pose.json)。其中 Standard seed 不能作为 MoveJ 执行角；
IK 服务会根据目标、执行角和臂角重建自己的 Standard 历史，不需要修改 C++ 初值。
**当前播放器不会自动读取 pose.json。原始矩阵 CSV 不含 IK 初值，因此今后换姿态必须
同时调整对应默认初值。`--input` 和 `ARM_TELEOP_TRAJECTORY_CSV` 只覆盖路径。**

## 设备更新和运行（由现场人员操作）

上位机更新 `for_cjp`，确认实际 `rosrun` 使用的是该仓库。此次只有 Python/输入文件
调整，源码工作空间通常无需重编译 C++；如果使用安装空间或复制部署，必须同步更新
实际运行的脚本、core 模块和 experiments 目录。

运动学库沿用已完成上一轮 R80 实验的冻结版本。Mac 本次数值验证提交为
`0b9ea8f1e714b79e7db8a90cc25b8c02c55c13d2`，`feature/redundancy-selector`；
没有新增库算法或配置修改需求。应保留已统一的关节限位，不能回到旧配置。
下位机沿用包含 tele 实际位置初始化、旧目标清除和电机13方向修正的
`feature/wrist-redundancy-hardware-validation`（Mac 本地基准 `ecac53d`）。
这些是代码要求，不是对设备实际安装状态的确认。

播放器终端先进入工作空间并清除旧 CSV 环境覆盖：

```bash
cd /home/pangu/pangu
source /opt/ros/noetic/setup.bash
source devel/setup.bash
unset ARM_TELEOP_TRAJECTORY_CSV
rosrun arm_teleop playback_right_teleop.py --preflight
```

核对显示的新 CSV 名称、1186 帧、39.5 秒及上述哈希。另一个完成同样 source 的终端，
启动右臂 IK 服务；若原 launch 已启动该服务，不要重复启动：

```bash
rosrun arm_teleop ik_service_right_node
```

先逐组做仅 IK 计算，不需要下位机，不发送 MoveJ 或 tele；每组重新启动 IK 服务，
让比较从独立历史开始：

```bash
rosrun arm_teleop playback_right_teleop.py --ik-only --ik-method A1_minimum_jv
rosrun arm_teleop playback_right_teleop.py --ik-only --ik-method minimum_sufficient_continuity_refined
```

上述两条分次执行。检查设备端结果后，现场确认新姿态的 MoveJ 接近路径、工具/线缆、
负载空间及停止措施。15 秒保持段不是 MoveJ 到位或安全保证。以下命令会实际运动，
必须分别执行，不是无人值守连跑；每组从同一构型及独立 IK 历史开始：

```bash
rosrun arm_teleop playback_right_teleop.py --ik-method A1_minimum_jv
rosrun arm_teleop playback_right_teleop.py --ik-method minimum_sufficient_continuity_refined
```

按两策略 × 有/无负载 × 每组5次采集。每次配对保存上位机 `online_right_teleop` CSV、
audit JSON、下位机 `tele_ft`；登记策略、负载、重复编号、代码版本、CSV 哈希和异常。
不能将数值通过解释为电流平台消失、碰撞检查通过或实机精度保证。

## Mac 验证

源验证：`analysis/trajectory_search/r80_clearance_20260923/`（项目工作区）。
该工程 CSV 经实际上位机映射后，两方法各1186帧，运动段675/675有效，预算/限位违规0，
异常HOLD 0，静止漂移0；另有9组小扰动全部通过。离线关闭deadline，未验证设备实时性。

本次部署修改还测试 CSV 哈希、初值与 pose.json 一致性、播放器实际初始化、
MoveJ 使用同一初值、完整路径映射、采样/保持段、旧轨迹回归及统一限位。
2026-09-24 本地结果：相关 ROS-free 测试 29/29 通过（警告视为错误）。已有两方法
完整数值输出各1186帧也逐帧通过本次上位机的限位、步长和速度校验；A1/MSC 最大
步长分别为 0.00423715/0.00423321 rad，峰值速度为 0.127115/0.126996 rad/s。
这是部署一致性复核，不是新运行的 ROS 在线或实机测试。
ROS-free 测试示例（仓库根目录）：

```bash
PYTHONPATH=vptele python3 -m unittest discover -s test -p test_r80_pose_300413.py -v
```

没有在 Mac 启动 ROS 服务、连接设备或执行实机动作。
