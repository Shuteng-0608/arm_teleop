# 脚本化 Peg-in-Hole 插入 — 工作总结

## 目标

在 MuJoCo 仿真中实现自动 peg-in-hole 插入控制器，复用 ROS IK 服务 + HDF5 记录器，生成与人遥操作格式一致的数据，用于对比实验。

## 新增文件

| 文件 | 说明 |
|------|------|
| `vptele/arm_control/scripted_insertion.py` | 核心控制器：三阶段状态机、标定、Jacobian IK、重力补偿 |
| `vptele/arm_control/scripted_insertion_ros.py` | ROS 外挂节点：Trigger 服务、episode 生命周期、HDF5 管理 |

## 修改文件

| 文件 | 改动 |
|------|------|
| `vptele/config/config_arm_right_peg.yaml` | 新增 `scripted_controller` 配置段（~60 行）；`vp_enabled: false`；`task_success_terminal_hold_time: 5.0`；`reset_arm_on_stop: false`；`monitor_camera_names` 加入三个摄像头 |
| `vptele/core/teleop_system_mujoco.py` | 新增 `_init_visionpro()`（VP 失败不阻塞）、`_initialize_scripted_controller()`；`vp_enabled` 判断跳过 VP |
| `vptele/arm_control/robot_controller_mujoco_peg_tool_contact.py` | 修复 4 处 import `vptele.utils` → `utils` |

## 整体流程

```
rosservice call /scripted_insertion/run "{}"
  │
  ├─ 1. reset_arm_to_initial_pose()
  ├─ 2. 固定孔位中心
  ├─ 3. 标定 IK↔World（首次 ~30s，缓存到 .npz）
  ├─ 4. 采样随机 XY 误差
  ├─ 5. start_episode (HDF5)
  ├─ 6. APPROACH → ALIGN → INSERT
  ├─ 7. stop_episode
  └─ 8. 返回 success/failure
```

## 标定：IK ↔ World 坐标变换

- 方法：6 方向中心差分（±X, ±Y, ±Z 各 20mm）+ SVD 投影到 SO(3)
- 结果缓存到 `data/.../calib_ik_to_world.npz`，进程重启自动加载
- 标定在 `start_episode` 之前完成，不混入数据

## APPROACH：接近墙面

- 目标 = 孔口中心 + 随机世界 XZ 偏移
- 偏移：固定 8mm 半径，方向 0-360°均匀随机，四象限等概率
- 力阈值：每 episode 随机 [15N, 28N]
- 自适应步长：80mm 以上 3mm/步，40-80mm 1mm/步，20-40mm 0.3mm/步，<20mm 0.1mm/步
- 力超过阈值 → 进入 ALIGN

## ALIGN：对齐孔中心

分三步：
1. **回撤** 3-8mm（ROS IK），离开墙面
2. **ROS IK 粗对齐**（最多 3 次），收敛到 ~3mm
3. **MuJoCo Jacobian 精对齐**（最多 6 次）+ 重力补偿 + 6-DOF 方向约束，收敛到 <1mm

### MuJoCo Jacobian IK

- 在局部 MjData 副本上运行 DLS 逆运动学，零坐标系偏差
- 重力补偿：`ctrl = q_ik + tau_g/kp`，消除执行器稳态误差
- 6-DOF 雅可比（位置 + 方向），保持 peg 垂直于墙面

## INSERT：推入 + 螺旋搜索

- 每步沿 -Y 推 0.5-2mm（自适应：力小大步、力大小步）
- 力 >5N 或 10 步无进展 → 回撤 4mm + 围绕孔中心螺旋搜索
- 成功条件：系统 TaskSuccessAutoStop 触发（peg 距 hole_goal <3mm）
- 成功后保持 2s 再结束

## RETRACT：退 peg

- 插入成功后调用 `_retract_clear()`
- hard_set_qpos 跳变 80mm 离开墙面，避免复位时碰撞

## 数据记录

- 记录内容与人遥操作完全一致：
  - `observations/ee_pose` 30Hz
  - `observations/joint_pos/vel/torque` 30Hz
  - `observations/ft_wrench` 500Hz（重力补偿后）
  - `observations/images/ee_cam, base_top_cam` 30Hz 640×480
  - `action` 30Hz
- episode_metadata 标记 `collection_method: "scripted_closed_loop"`
- 含 `scripted_error_xy_mm`、`scripted_error_angle_deg`

## 关键参数（当前值）

| 参数 | 值 |
|------|----|
| 墙接触力 | [15N, 28N] 随机 |
| APPROACH 偏移 | 固定 8mm，方向随机 0-360° |
| 回撤量 | [3mm, 8mm] 随机 |
| 终端保持 | 5s |
| 关节速度限制 | 0.2 rad/s（默认，未修改） |
| 偏移方向象限 | Q1右上 / Q2左上 / Q3左下 / Q4右下（日志标注） |

## 多视角显示

三个摄像头独立窗口：`cctv_cam`、`ee_cam`、`base_top_cam`。
