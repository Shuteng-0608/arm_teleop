# R30 / 7790 本次实验部署说明

本目录与播放器的初始化配置一起提交；替换旧 22205 默认实验。只修改
`playback_right_teleop.py` 路径选择及其 R30 初始配置，不修改 `main_ros.py`、
坐标映射、IK 算法、MoveJ/tele 流程、左臂保持方式或驱动保护。

## 版本与输入

- 上位机：`arm_teleop` 的 `for_cjp` 分支，包含本目录和对应初始化更新的提交。
- 运动学：`feature/redundancy-selector`，至少包含限位统一提交
  `c2e2e053d62760f435474a574052a5ab3a645162`。若此前使用更早冻结标签，
  须更新并重新编译安装库；本次并未调整冻结算法和指标阈值。
- 下位机：`feature/wrist-redundancy-hardware-validation`，当前本地基准
  `ecac53d9d6a19d34ecc79aa1af8efd24df8b3b87`。保留启动初值修正及当前电机方向；
  本次换 CSV 不要求再修改下位机。
- 输入：本目录 `R30_pose_7790_engineering.csv`，772 帧，30 Hz，25.7 s。
- SHA256：`54410f14dbca6e609809c8dea3378ebba82a700db7e7d5efe25bc69a1ddbd91a`。
- CSV 已随上位机仓库提交，默认无需手动复制到运动学仓库或 `data_log`。

轨迹由独立的 `trajectory_generator/scripts/generate_engineering_csv.py` 生成，
使用 `xz_circle_r30_22205/circle_printer_lookahead.yaml` 的位置/时间规划，换为
7790 首帧姿态模板。与原工程 CSV 相比，仅右臂 3×3 旋转矩阵列改变，
位置、时间、左臂及其余列不变。本文件是生成物的逐字节副本，不是预计算关节轨迹。
Mac 数值来源：运动学仓库
`results/redundancy_selector/r30_pose_search_current_limits_20260921/`。

原始 VP 坐标内为 XZ 平面，沿现有上位机轴映射后是机器人 YZ 平面，半径 **30 mm**。
固定末端姿态，先中心到圆周、绕圆、再返回中心。0–15 s 保持，15–23.7 s 运动，
23.7–25.7 s 保持。不能把前面的保持段当作 MoveJ 到位/负载检查。

## 与 CSV 配套的初始化

`vptele/core/right_arm_trajectory.py`：

```python
R30_BALANCED_INITIAL_RIGHT_JOINTS = [
    0.0591851471685264, -0.4778074189685440, -0.2609128124230255,
    1.4799132169734923, -0.8665106234837223, -0.8948895987437119,
    0.1714909921362970,
]
R30_BALANCED_INITIAL_RIGHT_ARM_ANGLE = -0.7853981633974482
```

单位 rad。关节角是 Offset 执行解；臂角是 Standard 选择器参数 −45°，
不可换成 Offset IK/FK 的参考臂角。首帧机器人位置为 `[0.3011,-0.358,0.2282]` m，
姿态四元数 wxyz 为
`[-0.0845439162280057,0.8477020537874809,-0.1008531874967262,-0.5138892767951758]`。

今后换原始矩阵 CSV 时，须同时核对这两个初始化量。原始矩阵 CSV 没有 IK 初值。
`--input` 和 `ARM_TELEOP_TRAJECTORY_CSV` 仍可覆盖路径，但不会自动推算配套初值。
不要用旧 22205 CSV 搭配新默认值；旧 22205 初始 q2 为正，已不满足统一限位。

## 设备上由操作人员部署

1. 保存设备已有修改后，更新上述分支。运动学库若尚未包含限位统一修正，按原有
   构建方式重新编译安装；确认安装后的选择器 YAML 也更新，不能只更新源码。
2. 更新上位机配置并重新编译 `arm_teleop` 节点，重新 source 工作空间和重启 IK。
   核对实际加载的 `kinematics_config_path`（如曾覆盖）以及选择器配置路径，
   避免仍加载旧库或旧限位。七关节范围与下位机配置保持一致，不放宽限位。
3. 在播放器终端取消可能遗留的路径覆盖：

```bash
cd /home/pangu/pangu
source /opt/ros/noetic/setup.bash
source devel/setup.bash
unset ARM_TELEOP_TRAJECTORY_CSV
```

在另一个完成相同 source 的终端启动右臂计算服务（若已通过原 launch 启动，
不要重复启动同一个服务）：

```bash
rosrun arm_teleop ik_service_right_node
```

先完整运行仅计算模式，不要求下位机启动，不发送 MoveJ 或 tele：

```bash
rosrun arm_teleop playback_right_teleop.py --ik-only --ik-method A1_minimum_jv
rosrun arm_teleop playback_right_teleop.py --ik-only --ik-method minimum_sufficient_continuity_refined
```

确认输出路径为本目录新 CSV、772 帧、上述哈希；检查在线求解日志的失败/HOLD、
限位、连续性与首帧一致性。Mac 离线两组各 261/261 运动步通过不等于设备实时验证通过。

确认 MoveJ 接近过程、负载、现场空间和停止措施后，由操作人员分别执行两组：

```bash
# 注意：以下命令会实际驱动机械臂！每组均重新从同一初始构型开始。
rosrun arm_teleop playback_right_teleop.py --ik-method A1_minimum_jv
rosrun arm_teleop playback_right_teleop.py --ik-method minimum_sufficient_continuity_refined
```

两条命令应分次执行并检查结果，不是无人值守连续运行。流程仍为右臂 MoveJ →
首帧 IK → tele → 逐帧在线 IK 发布；左臂保持反馈位置，双臂通道格式不变。
上位机保存 `data_log/online_right_teleop_*`，下位机保存对应跟踪记录，随后传回 Mac 分析。

## 本地验证与边界

从上位机仓库可运行不依赖 ROS 的回归：

```bash
PYTHONPATH=vptele python3 -m unittest discover -s test -p test_r30_pose_7790.py -v
PYTHONPATH=vptele python3 -m unittest discover -s test -p test_joint_limit_policy.py -v
```

新测试校验输入哈希、采样、保持段、实际映射的首帧位置/姿态、半径/平面、
配套初始化及入口默认路径。本次只在 Mac 修改验证和 Git 同步；未连接设备，
未在 ROS 环境编译运行或确认硬件部署。数值关节合规不是推杆负载、碰撞或 MoveJ 安全认证。

本次本地结果：新轨迹 7 项、限位策略 5 项、坐标映射 5 项、关节回放 4 项，
共 21/21 通过（将警告视为错误）。另外，既有 A1/冻结方案离线回放各 772 帧输出，
全部通过当前上位机的关节限位、单步变化和速度检查；初始化及映射位置路径一致。
这是对已有完整回放的部署一致性复核，不是重新运行 ROS 在线回放。
