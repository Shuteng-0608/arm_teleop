
``` bash
conda activate vrtele
```

# VisionPro 机械臂遥操作系统（多臂/多末端/多数据源）

本项目是基于 Apple Vision Pro 头显与机械臂的遥操作系统，支持多机械臂、多种末端执行器（灵巧手/夹爪/无末端），并可灵活选择数据流来源（vuer 或 avp_stream）。适用于远程精准控制机械臂及末端执行器，广泛应用于远程操作、协作机器人等场景。

---

## 主要特性

- **多机械臂支持**：可同时控制多台机械臂，进程隔离，互不干扰。
- **多末端执行器**：支持灵巧手（hand）、夹爪（gripper）和无末端（none）三种模式，配置灵活。
- **多数据源适配**：可选择通过 vuer（VisionDataServer）或 avp_stream（原生VisionPro数据流）获取手部/头部位姿。
- **实时高频控制**：支持高频率（默认30Hz及以上）位置与姿态更新，运动平滑，低延迟。
- **安全限制**：内置位置、姿态、速度等多重安全限制，防止越界与危险操作。
- **模块化架构**：核心功能高度解耦，便于扩展新的末端、算法或数据源。
- **丰富日志系统**：多进程日志隔离，支持控制台与文件日志，便于调试与追溯。
- **可视化工具**：内置机械臂运动轨迹与姿态可视化模块。

---

## 目录结构

```
vptele/
├── main.py                  # 单机械臂入口
├── multi_main.py            # 多机械臂入口
├── core/                    # 核心功能
│   ├── teleop_system.py     # 遥操作系统主控
│   ├── vp_streamer_vuer.py  # vuer数据流适配
│   └── vp_streamer_avp.py   # avp_stream数据流适配
├── arm_control/             # 机械臂控制相关
│   ├── robot_controller.py
│   ├── algo_controller.py
│   └── arm_teleop.py
├── end_effectors/           # 末端执行器
│   ├── end_effector_base.py
│   ├── hand/
│   │   ├── hand_teleop.py
│   │   └── hand_controller.py
│   └── gripper/
│       ├── gripper_teleop.py
│       └── gripper_controller.py
├── utils/                   # 工具模块
│   ├── logger.py
│   ├── math_utils.py
│   └── visualization.py
├── teleop/
│   ├── vision_data_server.py    # vuer数据服务器
│   └── image_server/            # 相机图像服务器
│       └── image_server.py
├── config/                  # 配置文件
│   ├── config.yaml
│   ├── config_arm_left.yaml
│   ├── config_arm_right.yaml
│   └── multi_arm_config.yaml
└── ...
```

---

## 快速开始

### 1. 安装依赖

```bash
pip install numpy pyyaml matplotlib avp_stream Robotic_Arm pyrealsense2 opencv-python zmq
```

> **注意**：`avp_stream` 和 `Robotic_Arm` 需根据实际环境安装，部分依赖可能为私有包。

### 2. 配置系统

- **单臂**：编辑 `config/config.yaml`
- **多臂**：编辑 `config/multi_arm_config.yaml`，并为每台机械臂准备独立的配置文件（如 `config_arm_left.yaml`、`config_arm_right.yaml`）

主要配置项包括：

- VisionPro 数据流 IP、模式（vuer/avp_stream）
- 机械臂 IP、端口、型号、初始位姿
- 末端执行器类型（hand/gripper/none）及其参数
- 控制参数（缩放、平滑、频率、安全范围等）
- 日志参数

---

## 3. 启动 VisionPro 数据服务器（如使用 vuer）

**使用 vuer 方式时，需先启动图像服务器，再启动数据服务器：**

1. **先启动图像服务器**（用于将机器人头部相机画面推送到 VisionPro）：

   ```bash
   python teleop/image_server/image_server.py
   ```

   > 请确保已正确连接并配置好相机（支持 RealSense 或 OpenCV 兼容相机）。

2. **再启动 VisionPro 数据服务器**：

   ```bash
   python teleop/vision_data_server.py --port 5301
   ```

   > 只有先启动 image_server，才能在 VisionPro 端看到机器人头部相机画面。

---

## 4. VisionPro 端操作说明

### vuer 模式

1. **在 VisionPro 头显的 Safari 浏览器中访问**：

   ```
   https://10.16.121.135:8012/?ws=wss://10.16.121.135:8012
   ```

   > 地址请替换为实际服务器 IP 和端口。

2. **点击页面上的 `Enter` 按钮**，即可进入遥操作界面，看到机器人头部相机画面。

3. **注意**：点击 `Enter` 时，**请确保 VisionPro 佩戴者面朝的方向与机器人面朝的方向一致**，以保证空间映射正确。

---

### avp_stream 模式

1. **在 VisionPro 头显中打开 `stream tracking` 应用**。

2. **点击 `Start` 按钮**，开始数据流推送。

3. **注意**：点击 `Start` 时，**同样需要保证 VisionPro 佩戴者面朝的方向与机器人面朝的方向一致**。

---

## 5. 运行遥操作系统

#### 单机械臂

```bash
python main.py --config config/config.yaml
```

#### 多机械臂

```bash
python multi_main.py --config config/multi_arm_config.yaml
```

---

## 配置说明

### 单机械臂配置（config.yaml）

```yaml
vp_ip: "10.16.104.165"
vuer: true            # true: 使用vuer; false: 使用avp_stream
robot_ip: "192.168.1.18"
robot_port: 8080
end_effector: "hand"  # hand/gripper/none
arm_config:
  scaling_factor: 1.0
  smoothing_factor: 0.5
  x_range: [-0.5, -0.3]
  y_range: [-0.2, 0.2]
  z_range: [0.2, 0.5]
  # ... 其他参数
hand_config:
  max_hand_range: 65535
  hand_speed: 1000
gripper_config:
  max_width: 85
  default_speed: 100
logging:
  console_level: "info"
  file_level: "debug"
```

### 多机械臂配置（multi_arm_config.yaml）

```yaml
left: config/config_arm_left.yaml
right: config/config_arm_right.yaml
```

每个子配置文件结构同上，支持独立参数。

---

## 主要模块说明

- **main.py / multi_main.py**：系统入口，负责参数解析、配置加载、系统启动。
- **core/teleop_system.py**：遥操作系统主控，协调数据流、机械臂、末端执行器等模块。
- **core/vp_streamer_vuer.py / vp_streamer_avp.py**：分别适配 VisionDataServer 和 avp_stream 两种数据源。
- **arm_control/**：机械臂底层控制与算法（正逆解、运动学等）。
- **end_effectors/**：末端执行器（灵巧手/夹爪）控制与手势映射。
- **utils/logger.py**：多进程安全日志系统，支持前缀区分。
- **utils/visualization.py**：机械臂轨迹与姿态可视化。

---

## 日志与调试

- 日志文件保存在 `logs/` 目录，按进程/机械臂区分。
- 控制台日志级别可通过 `--log-level` 或配置文件调整。
- 详细调试信息请关注日志输出。

---

## 常见问题

- **无法连接机械臂/数据流？**  
  检查 IP、端口、网络连通性，确保依赖库已正确安装。
- **多臂冲突或日志混乱？**  
  确保每台机械臂配置文件独立，日志前缀不冲突。
- **末端执行器无响应？**  
  检查 `end_effector` 配置，及相关硬件/驱动是否正常。
- **VisionPro 无法看到机器人画面？**  
  请确认 image_server 已启动，并且 VisionPro 端访问的服务器地址正确。

---

## 联系与贡献

如需定制开发、功能扩展或技术支持，请联系项目维护者。欢迎提交 issue 或 PR 共同完善本系统！

---