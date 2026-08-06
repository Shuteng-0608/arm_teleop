# Marvin M6 Vision Pro 纯 Python 遥操仿真

## 结论

Windows 上使用以下入口，不需要 ROS、`roscore`、`rosrun` 或 ROS IK 服务：

```powershell
python vptele\main_mujoco_marvin.py --vp-ip <VISION_PRO_IP>
```

运行链为：

```text
Vision Pro / avp-stream
  -> ArmTeleopMujocoMarvin
  -> fx_kine.py + libKine.dll + ccs_m6_31.MvKDCfg
  -> RobotControllerMuJoCoPegTool
  -> pangu_all_right_marvin_m6.xml
```

MuJoCo 物理、position actuator/contact 控制、力反馈 HUD、Vision Pro 输入、厂商本地 IK，以及可选 HDF5 录制，都在同一个 Python 进程中运行。

## Windows 安装与启动

当前厂商库和配置位于：

```text
ik_lib\libKine.dll
ik_lib\ccs_m6_31.MvKDCfg
```

安装运行依赖：

```powershell
python -m pip install numpy PyYAML mujoco opencv-python h5py avp-stream
```

确认 `python` 是 64 位：

```powershell
python -c "import struct; print(struct.calcsize('P') * 8)"
```

启动前，先在 Vision Pro Tracking Streamer 中打开 tracking，并确保电脑能访问 Vision Pro IP。然后在仓库根目录运行：

```powershell
python vptele\main_mujoco_marvin.py --vp-ip 192.168.x.x
```

常用选项：

```powershell
# 同时打开交互式 MuJoCo viewer
python vptele\main_mujoco_marvin.py --vp-ip <IP> --viewer

# 把本次运行记录成一条 HDF5 episode
python vptele\main_mujoco_marvin.py --vp-ip <IP> --record --label marvin_trial_001

# 关闭本机相机窗口和 Vision Pro 视频回传
python vptele\main_mujoco_marvin.py --vp-ip <IP> --no-camera-windows --no-video-return
```

按 `Ctrl+C` 会依次停止遥操、排空 recorder/render worker，并关闭 Vision Pro 连接。

## 动态库选择

配置同时声明两个平台的运动学库：

```yaml
arm_config:
  marvin_ik_library_path_windows: "../../ik_lib/libKine.dll"
  marvin_ik_library_path_linux: "../../ik_lib/libKine.so"
```

纯 Python 入口会按 `sys.platform` 自动选择：Windows 使用 DLL，Linux 使用 SO。`libMarvinSDK.dll`/`.so` 是机器人控制 SDK，不导出此处需要的运动学 ABI，不能替代 `libKine`。

当前仓库的 64 位 `libKine.dll` 已在 Windows 上完成真实加载测试；以起手关节做 FK→IK 回解，最大关节误差约为 `3.3e-7 rad`。

如果 DLL 加载报依赖缺失，应把厂商依赖 DLL 放到 `ik_lib`、Python 可执行文件目录，或加入 Windows `PATH`。DLL 位数必须和 Python 位数一致。

## 模型、控制器与起手式

| 项目 | 值 |
| --- | --- |
| 模型 | `model/pangu_all_right_marvin_m6.xml` |
| controller 命令空间起手式 | `[-1.57, 1.57, 1.57, 1.6, -1.57, 0, 0] rad` |
| `arm_sign` | `[1, -1, 1, -1, 1, 1, 1]` |
| MuJoCo/厂商 IK 内部关节 | `[-1.57, -1.57, 1.57, -1.6, -1.57, 0, 0] rad` |

入口复用 `RobotControllerMuJoCoPegTool`，通过 XML 中七个 position actuator 驱动机械臂，保留接触动力学、限速、力感知降速、高力 hold、异步渲染和 HDF5 writer。`scripts/view_mujoco_model_marvin.py` 仅用于查看模型，不是遥操入口。

厂商 `.MvKDCfg` 的关节方向与 MuJoCo 内部关节一致。因此 SDK FK/IK 使用乘过 `arm_sign` 的内部关节；IK 返回后再转换回 controller 命令空间，controller 写入 MuJoCo 前会再次应用符号。不要直接把命令空间第 4 轴的 `+1.6 rad` 传给 SDK。

## 逆解多解选择

本地适配层 `vptele/arm_control/marvin_ik_solver.py`：

- 每周期把上一周期关节解作为 IK 参考；
- `marvin_zsp_type: 0` 选择与参考关节欧氏距离最近的解；
- 拒绝零解、不可达、奇异、关节超限和单周期跳变过大的结果；
- 外部使用 rad/m，厂商接口使用 degree/mm；
- 当前只遥操末端平移，姿态保持起始 FK 姿态。

因此冗余臂多解不是任意取第一组，而是按连续参考解选择，并由外层跳变检查兜底。

## 关键文件

| 文件 | 作用 |
| --- | --- |
| `vptele/main_mujoco_marvin.py` | Windows/Linux 纯 Python 主入口和关闭顺序 |
| `vptele/config/config_arm_right_peg_marvin.yaml` | 模型、起手式、符号和平台动态库配置 |
| `vptele/arm_control/arm_teleop_mujoco_marvin.py` | AVP 映射、标定、本地 IK 和连续性检查 |
| `vptele/arm_control/marvin_ik_solver.py` | 厂商 ABI 初始化、单位转换、FK/IK 和安全检查 |
| `vptele/arm_control/robot_controller_mujoco_peg_tool_contact.py` | MuJoCo 物理、渲染、力安全和 HDF5 controller |
| `ik_lib/fx_kine.py` | 厂商 ctypes Python 包装 |
| `model/pangu_all_right_marvin_m6.xml` | Marvin M6、peg、墙孔、相机和 FT 模型 |

## 首次联调检查

1. 确认 Vision Pro tracking 正常且 `right_wrist` 数据有效。
2. 先使用较小 `scaling_factor`，核对三个手部移动方向。
3. 核对 SDK FK 法兰与 MuJoCo `link_7` 表示同一坐标系。
4. 再检查 peg 接触、FT/HUD、视频回传和 `--record` 生成的 HDF5。
