# arm_teleop 项目交接说明

本文档面向后续 agent。它描述的是当前仓库代码的实际状态，而不是旧 README 所描绘的理想化框架。

本文最后一次按源码重新核验于 **2026-08-06**。核验范围包括主配置、MuJoCo/ROS 入口、遥操作映射、controller、异步 HDF5 recorder、孔位调度、IK 服务、主/候选模型、数据脚本和测试。稳定主线仍以 Linux + ROS1 为目标；Marvin M6 另有可在 Windows 运行的纯 Python 路径。

## 先给结论

这个仓库是一个 ROS1 `catkin` 包，包名为 `arm_teleop`。当前真正投入使用的主线是：

> 用 Apple Vision Pro 的右手腕位姿遥操作一台 7 自由度右臂，在 MuJoCo 中完成水平墙面插销入孔任务；同时渲染操作员视角、叠加力反馈 HUD，并把多相机图像、关节状态、执行器指令和六维力/力矩按 episode 写入 HDF5 数据集。

它同时保留了真实机械臂、双臂、灵巧手、旧 MuJoCo 控制器和旧数据格式相关代码，但这些不是当前插销入孔采集链的可靠入口，部分代码已经不完整或与当前 schema 不兼容。

当前主入口是 `vptele/main_mujoco.py`，主配置是 `vptele/config/config_arm_right_peg.yaml`，主模型是 `model/pangu_all_right.xml`。

## 当前主数据流

```text
Apple Vision Pro / Tracking Streamer
  -> avp_stream.VisionProStreamer
  -> vptele/core/vp_streamer_avp.py
  -> vptele/arm_control/arm_teleop_mujoco.py
       右手位姿增量 -> 固定姿态的末端目标 -> ROS ArmIK 服务
  -> src/ik_service.cpp
       目标末端位姿 -> 7 关节角
  -> RobotControllerMuJoCoPegTool.set_arm_positions()
  -> 关节符号变换 + 限速/力感知降速
  -> MuJoCo position actuators + mj_step()
       |
       +-> 500 Hz 力流、30 Hz 状态/动作流 -> 异步 HDF5 writer
       +-> 仿真状态快照 -> 独立 render worker
             +-> ee_cam/base_top_cam 原始 RGB -> HDF5
             +-> cctv_cam + 力反馈 HUD -> OpenCV 窗口
             +-> cctv_cam + 力反馈 HUD -> Vision Pro WebRTC
```

MuJoCo XML 的 `timestep` 是 `0.001 s`，所以物理目标频率是 1000 Hz。当前配置记录：

- 六维力/力矩：500 Hz；
- 关节、末端位姿和动作：30 Hz；
- `ee_cam`、`base_top_cam` 图像：30 Hz，640×480 RGB；
- 操作员 `cctv_cam`：15 Hz，最终画面适配到 1280×720；
- Vision Pro 回传端口：9999。

这些流使用各自的时间戳，不能按数组下标直接假设一一对应。

## 主运行链

### 1. 配置加载

`vptele/main_mujoco.py`：

1. 初始化 ROS 节点；
2. 从私有 ROS 参数读取 `config_path` 等覆盖值；
3. 用 `vptele/utils/mujoco_config.py` 严格加载 YAML；
4. 拒绝重复 YAML key，解析相对模型/数据路径，并执行跨字段校验；
5. 创建 `TeleopSystemMujoco`。

配置中的相对路径相对于配置文件所在目录解析。因此：

- `../../model/pangu_all_right.xml` 指向仓库根目录的 `model/pangu_all_right.xml`；
- `../../data/hole_random_60mm_hmj` 指向仓库根目录的 `data/hole_random_60mm_hmj`。

### 2. 系统初始化

`vptele/core/teleop_system_mujoco.py` 按下面顺序初始化：

1. 连接 `avp_stream`，等待有效 `right_wrist` 4×4 变换矩阵；
2. 创建 `RobotControllerMuJoCoPegTool`；
3. 可选启动 Vision Pro 单目 WebRTC 视频；
4. 创建 `ArmTeleopMujoco`，连接右臂 IK 服务并标定当前右手参考位姿；
5. 注册 episode 级遥操作服务；
6. 最后调用 controller 的 `activate_runtime()`，启动物理/渲染线程并发布录制服务。

录制服务最后发布是有意设计：这保证孔位、IK、遥操作服务、仿真和渲染都已经准备好。

### 3. Vision Pro 到机械臂的映射

`vptele/arm_control/arm_teleop_mujoco.py` 当前只使用右手平移增量。末端姿态映射代码被注释掉，机器人姿态始终保持 `arm_config.initial_robot_pose` 中的初始欧拉角。

当前平移映射为：

```text
robot X += hand Y * scaling_factor
robot Y += hand Z * scaling_factor * 0.8
robot Z += hand X * scaling_factor * 0.8
```

目标位姿经过 `PoseFilter7D`，随后用 `/arm_teleop/right_arm_ik_srv` 的 `optimal_ref` 方法求解。成功的关节角还会经过 `smooth_values()`，再送给 MuJoCo controller。控制循环周期配置为 `0.01 s`，但真实频率还受 ROS IK 调用耗时影响。

### 4. IK 服务

`src/ik_service.cpp` 发布：

```text
/arm_teleop/right_arm_ik_srv
```

服务类型是 `srv/ArmIK.srv`。它支持 `feasible_ref`、`feasible_std`、`optimal_ref`、`optimal_std`，并用 FK 回算检查 IK 结果。

注意：当前 CMake 实际使用外部 `ArmKinematics` 包和 `/usr/local/lib/libarm_kinematics.so`。仓库里的 `src/kinematics_*.cpp`、`lib/kinematics_*.h` 对应的本地静态库构建块已经被注释，当前不会参与 `ik_service_*` 的链接。

### 5. MuJoCo 控制

`vptele/arm_control/robot_controller_mujoco_peg_tool_contact.py` 是当前最重要、也最大的实现文件。

正常路径必须使用 `control_mode: actuator`：

- `set_arm_positions()` 只更新目标；
- controller 使用 `arm_sign = [-1, 1, 1, -1, 1, 1, 1]` 转成 MuJoCo 内部符号；
- 物理线程按速度上限把 `command_joints` 逐步靠近 `target_joints`；
- `data.ctrl` 驱动 position actuator；
- `mujoco.mj_step()` 真正计算动力学和接触。

不要把连续控制改回直接覆盖 `qpos`，否则接触将无法阻止插销穿透。`qpos` 直写只用于初始化、episode reset 和调试模式。

当前安全策略包括：

- 最大关节速度 `0.2 rad/s`；
- 力范数达到 40 N 后速度缩放到 0.4；
- 达到 80 N 后缩放到 0.15；
- 达到 100 N 并持续 0.15 s 后进入 high-force hold，低于 80 N 释放；
- 关节力矩报警阈值 `[30, 30, 20, 20, 10, 10, 10] Nm`；
- FT 报警阈值 100 N / 3 Nm，持续 0.25 s。

## Episode 生命周期

当前配置 `teleop_controlled_by_recording: true`。系统启动后物理仿真会运行，但遥操作线程不会立即运行；episode 由录制服务控制。

录制服务：

```text
/mujoco_hdf5_recording/set_recording
srv/SetRecording.srv
```

开始 episode 时依次执行：

1. 拒绝旧遥操作命令；
2. 停止旧遥操作线程；
3. 把机械臂硬复位到 `initial_arm_joints`；
4. 清空关节力矩、FT 和任务成功状态；
5. 重新标定当前 Vision Pro 右手为零点；
6. 用当前孔位 metadata 创建 HDF5；
7. 允许 controller 接收命令；
8. 启动新的遥操作线程。

手动停止时依次执行：

1. 立即拒绝新命令；
2. 停止遥操作线程；
3. 完成并关闭 HDF5；
4. 在录制已经停止后复位机械臂，避免 reset 污染数据；
5. 根据 `keep` 保留或删除整个 episode 目录；
6. 保留则推进孔位，丢弃则重试当前孔位。

操作端可以另开终端运行：

```bash
rosrun arm_teleop recording_keyboard_client.py
```

回车开始，再次回车时选择保留或丢弃。

### 自动成功停止

成功条件由 `peg_tip_site` 和 `hole_goal_site` 的世界坐标距离决定。当前配置要求：

- 距离不超过 3 mm；
- 连续满足 0.10 s；
- 成功后用 0.2 s 把 actuator command 混合到当前实际 `qpos`；
- 保持 1.0 s 后自动停止 HDF5。

自动停止的 episode 会先临时保留，等待键盘客户端人工审核。审核“保留”才推进孔位；“丢弃”会删除目录并重试同一孔位。

## 孔位采样

当前使用 5×5 X-Z 网格：

- X 和 Z 都覆盖 `[-0.060, 0.060] m`；
- 默认取每格中心，即 `[-48, -24, 0, 24, 48] mm`；
- Y 偏移为 0；
- 每个 cycle 用 seed 42 确定性打乱；
- 每个 cycle 的 25 个格子各出现一次；
- `hole_grid_start_cycle` 和 `hole_grid_start_index` 都是从 0 开始的恢复游标。

调度器在 `vptele/utils/hole_grid_scheduler.py`，更完整的采集约定在 `docs/hole_grid_collection.md`。网格 metadata 会写入 HDF5 和同目录的 `metadata.json`。

当前模型中：

- 插销半径 0.011 m；
- 插销长度 0.090 m；
- 孔半径约 0.014 m；
- 孔深约 0.045 m；
- 插入方向是世界坐标/任务坐标的负 Y。

孔尺寸不是在 recorder 中硬编码的；recorder 会从 `cylindrical_peg` 和 24 个 `wall_hole_ring_*` geom 推断。

## 渲染与线程所有权

这是后续修改最容易破坏的部分。

### 物理线程

`visualization_thread()` 是 live `self.data` 的主要所有者。每次 `_physics_step()` 在同一个 `RLock` 内完成：

1. 力感知限速或 hold；
2. 写 actuator target；
3. `mj_step()`；
4. 更新报警和任务成功状态；
5. 复制应记录的数值样本；
6. 复制一份不可变渲染快照并尝试放入 render queue。

不要在这里做 OpenGL 渲染、`cv2.imshow`、WebRTC 阻塞发送或 HDF5 磁盘写入。

### 渲染线程

`MujocoRenderWorker` 有自己的浅拷贝 `MjModel` 和独立 `MjData`。它根据物理线程复制出的快照恢复场景，再渲染：

- 数据集相机；
- 操作员相机；
- Vision Pro 回传帧。

队列满时会丢渲染请求而不是阻塞物理线程；丢帧数写入 episode metadata，并在 shutdown 输出 runtime metrics。

操作员 HUD 只叠加在 live CCTV 帧上。写入 HDF5 的 `ee_cam`、`base_top_cam` 是无 HUD 的原始 RGB 帧，这是有意的数据隔离。

### HDF5 写线程

`MujocoHDF5Writer` 独占 steady-state h5py append：

- force 行每 64 条批量写；
- state 行每 16 条批量写；
- 图像由 render worker 提交后异步写；
- stop 时等待尚未完成的图像请求，插入 barrier，flush、裁掉预分配行，再写最终 metadata。

保持“物理线程只复制数据、渲染线程只渲染、writer 线程只写盘”的边界。关闭顺序也不能随意调整：先停物理线程，再让 recorder 排空请求，最后由 render worker 关闭自己的 OpenGL/OpenCV 资源。

## 当前 HDF5 schema

schema 版本是 `compact_mujoco_hdf5_v1`。每条保留 episode 形如：

```text
data/hole_random_60mm_hmj/<timestamp>_<label>/
  episode.hdf5
  metadata.json
```

主要数据集：

```text
timestamps/state
timestamps/state_episode
timestamps/force
timestamps/force_episode
timestamps/image
timestamps/image_episode

observations/ee_pose                 [N_state, 7]  xyz + qw qx qy qz
observations/joint_pos               [N_state, 7]
observations/joint_vel               [N_state, 7]
observations/joint_torque            [N_state, 7]
observations/ft_wrench               [N_force, 6]  重力补偿后
observations/ft_wrench_raw           [N_force, 6]
observations/ft_wrench_gravity       [N_force, 6]
observations/images/ee_cam           [N_image, H, W, 3] uint8 RGB
observations/images/base_top_cam     [N_image, H, W, 3] uint8 RGB

actions/joint_pos_command            [N_state, 7]  data.ctrl 中的实际 position target
action                               [N_state, 7]  ACT 兼容别名

events/names
events/t_sim
events/t_episode
events/t_wall
events/extra_json

episode_metadata/...                 初末状态、名称、采样率、孔位和任务几何
```

`observations/ft_wrench` 的当前约定是：

```text
ft_wrench = ft_wrench_raw - ft_wrench_gravity
```

六维量顺序为 `[Fx, Fy, Fz, Tx, Ty, Tz]`，表达在 `ft_sensor_site` 坐标系。所有跨流对齐都应基于 `timestamps/*`。

## ROS 接口汇总

当前主线会用到：

```text
/arm_teleop/right_arm_ik_srv                 arm_teleop/ArmIK
/arm_teleop_mujoco/stop                      std_srvs/Trigger
/arm_teleop_mujoco/recalibrate               std_srvs/Trigger
/arm_teleop_mujoco/start                     std_srvs/Trigger
/mujoco_hdf5_recording/set_recording          arm_teleop/SetRecording
```

`launch/teleop_service.launch` 会同时启动左右 IK 服务；当前插销任务只需要右侧服务。

## 关键文件

| 文件 | 作用 |
| --- | --- |
| `vptele/main_mujoco.py` | 当前 ROS Python 入口 |
| `vptele/config/config_arm_right_peg.yaml` | 当前唯一可信的完整运行配置 |
| `vptele/main_mujoco_marvin.py` | 实验性 Marvin M6 纯 Python 入口；不需要 roscore 或 ROS IK/episode 服务 |
| `vptele/config/config_arm_right_peg_marvin.yaml` | 继承主配置并覆盖 Marvin 模型、起手式、关节符号和本地 IK |
| `vptele/arm_control/arm_teleop_mujoco_marvin.py` | Marvin 的 AVP 映射、坐标标定和本地 IK 控制循环 |
| `vptele/arm_control/marvin_ik_solver.py` | `fx_kine.py` 的 rad/degree、m/mm 和错误处理适配层 |
| `vptele/core/teleop_system_mujoco.py` | 组件装配和启动/关闭顺序 |
| `vptele/core/vp_streamer_avp.py` | `avp_stream` 适配和 CCTV WebRTC 回传 |
| `vptele/arm_control/arm_teleop_mujoco.py` | 手腕映射、滤波、IK 调用、episode 遥操作服务 |
| `vptele/arm_control/robot_controller_mujoco_peg_tool_contact.py` | 物理、控制、安全、渲染、episode 和录制协调 |
| `vptele/utils/mujoco_hdf5_recorder.py` | 当前 compact HDF5 schema 和异步 writer |
| `vptele/utils/force_feedback_overlay.py` | 力反馈数值、状态判定和 HUD 绘制 |
| `vptele/utils/ft_wrench_utils.py` | FT 原始值和重力补偿计算 |
| `vptele/utils/hole_grid_scheduler.py` | 可复现 5×5 孔位调度 |
| `vptele/utils/mujoco_config.py` | 严格配置加载、路径归一化和校验 |
| `model/pangu_all_right.xml` | 当前 7 轴、peg、墙孔、FT、5 相机模型 |
| `src/ik_service.cpp` | 当前右臂 ROS IK 服务 |
| `scripts/check_hdf5_episode_quality.py` | 当前 schema 的 episode 质量检查和网格覆盖统计 |
| `scripts/recording_keyboard_client.py` | episode 人工开始、保留、丢弃客户端 |
| `docs/hole_grid_collection.md` | 网格采集规则 |
| `docs/peg_in_hole_visual_force_dataset_strategy.md` | 数据集与训练策略背景，不是运行规范 |

### 尚未接入主线的 Marvin M6 候选资产

仓库里还有一条独立的实验性 Marvin M6 路径。代码已经接线，但厂商运行资产不完整，不能取代当前稳定主线：

| 文件 | 当前状态 |
| --- | --- |
| `model/pangu_all_right_marvin_m6.xml` | 由 `config_arm_right_peg_marvin.yaml` 引用，保留主任务所需的关节/actuator、相机、FT、peg 和墙孔命名 |
| `Marvin M6-S-R-CCS-696-V4.0 urdf (FUSION)/` | Marvin M6 的 URDF、MuJoCo XML 和网格源资产；候选模型会加载其中的 mesh |
| `scripts/view_mujoco_model_marvin.py` | 只用于查看候选模型；默认模型路径硬编码为本仓库的 Windows 绝对路径，跨机器必须传 `--model` |
| `ik_lib/fx_kine.py` | 被 Marvin 适配层按文件路径加载的厂商运动学 ctypes 封装 |
| `ik_lib/libKine.dll` / `libKine.so` | Marvin 运动学 ABI；纯 Python 入口在 Windows/Linux 自动选择 |
| `ik_lib/libMarvinSDK.dll` / `.so` | 机器人控制 SDK，不能替代 `libKine` |

Marvin 在 Windows 上用 `python vptele\main_mujoco_marvin.py --vp-ip <IP>` 直接启动。入口自动选择 `libKine.dll`；Windows 64 位 Python 已完成 DLL 真实加载和起手式 FK→IK 回解验证。完整设计和首次联调清单见 `docs/marvin_mujoco_teleop.md`。

不要仅把稳定主 YAML 的 `mujoco_model_path` 改成 Marvin 模型。Marvin 必须走独立 Python-only 配置和本地 `fx_kine.py` 适配层；联调时还要核对 `.MvKDCfg` 型号、FK 起始位姿、`arm_sign`、关节限制、actuator 力矩、peg 安装变换、FT 重力补偿和成功判定。

## 预期构建与运行环境

目标环境是 Linux + ROS1，不是当前 Windows 工作区本身。典型依赖：

- ROS1/catkin、`roscpp`、`rospy`、`geometry_msgs`、`sensor_msgs`、`trajectory_msgs`、`std_srvs`；
- Eigen3、Ceres、yaml-cpp；
- 已安装的 `ArmKinematics` CMake package 和 `/usr/local/lib/libarm_kinematics.so`；
- Python：NumPy、SciPy、PyYAML、MuJoCo、h5py、OpenCV、Matplotlib、`avp-stream`；
- 图形或 offscreen OpenGL 环境。

`config/environment.yaml` 和 `vptele/environment.yaml` 来自不同机器/架构，而且都没有完整列出当前 MuJoCo/HDF5 依赖，不能视为可复现 lockfile。前者是 Linux x86_64/Python 3.13 痕迹，后者包含 Linux aarch64/Python 3.9 痕迹。`package.xml` 也没有完整声明 CMake/Python 实际使用的全部依赖，因此迁移环境时要以 import、CMake 和运行链的并集为准。

CMake 还硬编码了：

```text
/usr/local/lib/libarm_kinematics.so
/usr/lib/libceres.so
```

迁移机器时应先检查这些路径。

典型启动方式：

```bash
# catkin workspace 根目录
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3
source devel/setup.bash

# terminal 1
roscore

# terminal 2：当前任务只需右臂
rosrun arm_teleop ik_service_right_node

# 在 Vision Pro Tracking Streamer 中先启动 tracking，并确认配置的 vp_ip 可达

# terminal 3
rosrun arm_teleop main_mujoco.py \
  _config_path:=config/config_arm_right_peg.yaml

# terminal 4
rosrun arm_teleop recording_keyboard_client.py
```

默认配置里的 `vp_ip: 192.168.1.144` 是现场值，换网络时必须修改或用 ROS 私有参数覆盖。`main_mujoco.py` 只支持 `mode="full"`。

## 测试和数据工具

当前有价值的单元/集成测试：

```text
test/test_mujoco_config.py
test/test_hole_grid_scheduler.py
test/test_hole_grid_integration.py
test/test_hole_grid_coverage.py
test/test_mujoco_hdf5_async.py
test/test_visionpro_video.py
```

可在依赖完整的 Linux 环境运行。由于仓库的 `test/` 没有 `__init__.py`，使用 discovery 比 `python3 -m unittest test.test_...` 更稳妥：

```bash
for pattern in \
  test_mujoco_config.py \
  test_hole_grid_scheduler.py \
  test_hole_grid_integration.py \
  test_hole_grid_coverage.py \
  test_mujoco_hdf5_async.py \
  test_visionpro_video.py
do
  python3 -m unittest discover -s test -p "$pattern" -v || exit 1
done
```

`test/test_ik.py` 需要 ROS master 和已经运行的 IK 服务。

查看模型：

```bash
python3 scripts/view_mujoco_model.py --model model/pangu_all_right.xml
```

检查单条当前 HDF5：

```bash
python3 scripts/check_hdf5_episode_quality.py \
  data/hole_random_60mm_hmj/<episode>/episode.hdf5 \
  --forcerange 30,30,20,20,10,10,10
```

注意质量脚本自己的默认 `--forcerange` 仍是 `20,20,20,20,10,10,10`，与当前模型前两个 actuator 的 30 Nm 不一致，所以应显式传参。

检查 5×5 覆盖：

```bash
python3 scripts/check_hdf5_episode_quality.py \
  --coverage-root data/hole_random_60mm_hmj \
  --grid-target-per-cell 10
```

## 已知不一致和陷阱

1. 根 `README.md` 只有三个最小启动步骤，缺少配置、Vision Pro、录制和环境约束；`vptele/README.md` 则明显滞后，仍描述已经缺失的 `main.py`、`multi_main.py`、gripper 等模块。两者都不应作为当前架构的 source of truth。
2. `vptele/main_ros.py`、`core/teleop_system_ros.py` 和 `arm_control/arm_teleop_ros.py` 是历史真实机器人/双臂路径。当前 `TeleopSystemROS._initialize_robot_controller()` 没有实际创建 controller，默认配置文件也不存在；`main_ros.py` 还会把 VP IP 强制覆盖成 `192.168.8.145`。不要宣称这条路径可直接运行。
3. 灵巧手代码依赖仓库外的 `aiui` ROS 服务；真实臂路径还依赖仓库外的 `arm_angle`。当前 peg 配置不会初始化手。
4. `vptele/core/vp_streamer_vuer.py` 是旧路径，内部把 server URL 硬编码为 `http://localhost:5301`，MuJoCo 主线也不会选择它。
5. Python import 同时出现 `utils.*`、`core.*` 和 `vptele.utils.*`。这依赖 catkin/源码目录的 `PYTHONPATH` 布局；优先使用 `rosrun`，不要未经验证就改成 `python -m vptele...`。
6. `setup.py` 的 package discovery 和注释较混乱，且 `vptele/` 根目录没有普通 `__init__.py`。改打包方式前必须同时验证 catkin devel/install space 的 import。
7. `scripts/analyze_mujoco_hdf5_episode.py` 仍读取旧字段，如 `observations/qpos`、`observations/force_torque`、`actions/q_cmd`；不兼容当前 compact schema。
8. `scripts/trim_mujoco_hdf5_episode.py` 的状态字段和“每相机子组 timestamps”也来自旧 schema；不能直接用来裁当前 episode。
9. `vptele/utils/mujoco_hdf5_recorder_old.py`、`robot_controller_mujoco.py`、`model/right_arm_stable.xml` 等是兼容/历史文件，不是当前 source of truth。
10. 当前工作区没有 `.git` 目录，`git status` 不可用；这是一个源码快照。不要假设能查看历史或提交。
11. 当前工作区的 Windows Python 3.13 有 NumPy，但没有 PyYAML、MuJoCo、h5py、OpenCV 和 `avp-stream`，因此尚不能做 Vision Pro/MuJoCo 端到端运行。安装这些 Python 包后，Marvin Python-only 入口可以直接在 Windows 运行；稳定 ROS 主线仍以 Linux/ROS 为目标。
12. `.gitignore` 忽略 `data/`、HDF5、图片和日志，因此采集结果通常不会随源码流转。
13. Marvin M6 已有独立纯 Python 入口和配置，不需要 ROS。入口在 Windows 选择 `libKine.dll`、在 Linux 选择 `libKine.so`。Windows 64 位 Python 对 DLL 的真实加载、配置初始化和起手式 FK→IK 已通过，最大回解误差约 `3.3e-7 rad`；`libMarvinSDK` 仍不能替代运动学库。
14. `scripts/view_mujoco_model_marvin.py` 的默认模型参数是 Windows 绝对路径；它仍只做模型查看。真正遥操应使用 `main_mujoco_marvin.py`，不要把查看脚本改造成控制主循环。
15. `package.xml` 的依赖声明不完整：例如代码/CMake 还会用到 `geometry_msgs`、`roslib`、`std_srvs`、Eigen3、Ceres、ArmKinematics 及多项 Python 包。不能只依赖 `rosdep` 或现有 environment YAML 推断出完整环境。
16. `srv/ArmIK.srv` 对 `method` 的注释仍写着旧选项；当前 `src/ik_service.cpp` 的实际有效值是 `feasible_ref`、`feasible_std`、`optimal_ref`、`optimal_std`，MuJoCo 主线发送 `optimal_ref`。

本次在 Windows 快照上完成的验证包括：所有 Python 文件通过语法编译，`model/*.xml` 可作为 XML 解析，两个模型保留预期接口，Marvin IK 适配层和 Python-only 配置测试通过，`libKine.dll` 的真实加载、配置初始化和起手式 FK→IK 回解通过。MuJoCo/OpenGL、Vision Pro 和 HDF5 异步链仍需在安装完整依赖后联调。

## 后续修改时的约束

- 修改主链前先判断需求属于“当前 MuJoCo peg 数据采集”还是“历史真实臂/灵巧手”。
- 修改 controller 配置时同步检查 `mujoco_config.py` 的校验、`config_arm_right_peg.yaml` 和 `test_mujoco_config.py`。
- 修改 XML 中关节、actuator、site、sensor、camera 或 hole geom 名称时，同步检查 controller、recorder、质量脚本和集成测试中的字符串引用。
- 修改 HDF5 字段时应更新 schema version、`schema_json`、质量检查脚本和 async recorder 测试；不要只改 recorder。
- 修改 episode 开始/停止顺序时，必须保证 reset 动作不进入保留数据。
- 修改渲染时保持 live CCTV/HUD 与训练图像分离，除非需求明确要求把 HUD 写入数据集。
- 修改线程代码时优先保持非阻塞物理循环，并保留 render drop、writer flush 和安全关闭行为。
- 判断“是否成功插入”时以 `peg_tip_site` 到 `hole_goal_site` 的距离和 dwell 为准，不要把可视 marker 当成任务真值。
- 修改 Marvin 路径时同时核对 `main_mujoco_marvin.py`、`config_arm_right_peg_marvin.yaml`、`arm_teleop_mujoco_marvin.py`、`marvin_ik_solver.py` 和 `docs/marvin_mujoco_teleop.md`。完成 Vision Pro/MuJoCo 实机联调前不要把它标成稳定主线。

如果代码与本文档冲突，可信顺序是：当前主配置和测试 > 当前主线实现 > 本文档 > 两份 README/旧脚本。
