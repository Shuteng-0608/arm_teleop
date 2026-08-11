#!/usr/bin/env python3
"""
测试脚本：把 peg 直接移到孔口中心，验证坐标转换是否正确。

用法: python3 test_hole_center.py
前提: 先启动 main_mujoco.py (仿真+ROS IK)
"""

import time
import numpy as np
import rospy
from arm_teleop.srv import ArmIK, ArmIKRequest
from scipy.spatial.transform import Rotation as R
import mujoco


# 加载标定数据
calib = np.load(
    "/home/hmj/pangu/src/arm_teleop/data/hole_random_60mm_hmj/calib_ik_to_world.npz")
R_w_ik = calib["R"]
t_w_ik = calib["t"]
print(f"R_w_ik =\n{R_w_ik}")
print(f"t_w_ik = {t_w_ik}")


def world_to_ik(p_w):
    return R_w_ik.T @ (p_w - t_w_ik)


# 提供 robot_controller 的替代品
# 直接读取 MuJoCo 模型获取 site 位置
model_path = "/home/hmj/pangu/src/arm_teleop/model/pangu_all_right.xml"
model = mujoco.MjModel.from_xml_path(model_path)
data = mujoco.MjData(model)

# 读 hole_goal 和 wall_task 位置（需要仿真运行中，sim 线程在更新 data）
# 这里只能从 model 读取 body_pos
hole_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "wall_task")
hole_goal_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "hole_goal_site")

print(f"\n=== MuJoCo Model 静态数据 ===")
print(f"wall_task body_pos = {model.body_pos[hole_body_id]}")
print(f"hole_goal_site pos (relative to wall_task) = {model.site_pos[hole_goal_id]}")

# hole_goal 的世界位置 = body_pos + site_pos (需要 mj_forward)
mujoco.mj_forward(model, data)
hole_goal_w = data.site_xpos[hole_goal_id].copy()
wall_w = data.xpos[hole_body_id].copy()
print(f"hole_goal world = {hole_goal_w}")
print(f"wall_task world = {wall_w}")

# 孔口 = hole_goal XZ + wall Y
hole_entrance = hole_goal_w.copy()
hole_entrance[1] = float(wall_w[1])
print(f"hole_entrance world = {hole_entrance}")

# 转 IK 坐标
hole_entrance_ik = world_to_ik(hole_entrance)
print(f"hole_entrance IK = {hole_entrance_ik}")

# 参考 IK 位置
ik_ref = np.array([0.3011, -0.358, 0.2282])
print(f"\n=== 对比 ===")
print(f"IK 参考位置: {ik_ref}")
print(f"孔口 IK 位置: {hole_entrance_ik}")
print(f"差值: {hole_entrance_ik - ik_ref}")
print(f"距离: {np.linalg.norm(hole_entrance_ik - ik_ref)*1000:.1f}mm")
