#!/usr/bin/env python3
import time
import math
import numpy as np
import mujoco

def run_headless_benchmark():
    # 1. 模型路径
    model_path = "/home/hmj/pangu/src/arm_teleop/model/pangu_all_right.xml"
    print(f"正在加载模型: {model_path}")
    
    try:
        model = mujoco.MjModel.from_xml_path(model_path)
        data = mujoco.MjData(model)
        print("模型加载成功！")
    except Exception as e:
        print(f"模型加载失败，请检查路径: {e}")
        return

    sim_timestep = model.opt.timestep  # XML 里是 0.001s (1000Hz)
    print(f"MuJoCo 物理步长 (timestep): {sim_timestep}s ({1/sim_timestep:.0f} Hz)")
    print(f"模型包含执行器数量: {model.nu}")

    # 初始关节位置
    q0 = np.array([-0.046, -0.2, 0.0, 1.6, -1.32, 0.005, 0.005])
    
    # 写入初始位置
    for i in range(7):
        joint_name = f"joint_{i+1}"
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
        if joint_id != -1:
            qpos_addr = model.jnt_qposadr[joint_id]
            data.qpos[qpos_addr] = q0[i]
            
    mujoco.mj_forward(model, data)
    print("机械臂已初始化到默认姿态。")
    print("\n" + "="*50)
    print("开始纯物理引擎计算基准测试 (无头 Headless 模式)...")
    print("测试将运行 30000 步物理推进，相当于仿真时间轴中的 30 秒。")
    print("="*50 + "\n")

    # 总计测试步数（30000步 @ 1000Hz = 实际30秒的物理时间）
    total_steps = 30000
    
    # 记录真实世界的时间戳
    wall_start_time = time.perf_counter()
    
    for step in range(total_steps):
        # 模拟控制循环：发送 0.5Hz 的正弦命令
        t_sim = data.time
        target_command = q0 + 0.15 * math.sin(2 * math.pi * 0.5 * t_sim)
        
        for i in range(7):
            actuator_name = f"motor_joint_{i+1}"
            actuator_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_name)
            if actuator_id != -1:
                data.ctrl[actuator_id] = target_command[i]
                
        # 纯数值向前推一步物理（不包含显卡画面交换）
        mujoco.mj_step(model, data)

    wall_end_time = time.perf_counter()
    
    # 性能计算
    total_wall_time = wall_end_time - wall_start_time
    steps_per_second = total_steps / total_wall_time
    realtime_factor = (total_steps * sim_timestep) / total_wall_time

    print("测试完成！运行报告如下：")
    print(f"* 成功仿真了 (模拟时间): {data.time:.2f} 秒")
    print(f"* 实际花费时间 (真实世界): {total_wall_time:.4f} 秒")
    print(f"* 纯算法物理迭代频率: {steps_per_second:.2f} Hz")
    print(f"* 实时加速比 (Real-time Factor): {realtime_factor:.2f} x")
    
    print("\n" + "-"*50)
    if steps_per_second > 1000:
        print("【诊断结论】: CPU 计算性能极度充沛！纯物理能远超 1000Hz。\n"
              "这证实了之前的卡顿完全是由显卡窗口渲染交换（V-Sync/X11/Wayland 锁频）导致的。")
    else:
        print("【诊断结论】: 纯物理计算也低于 1000Hz。说明当前模型的 xml 物理参数配置过重，\n"
              "或者 CPU 存在单核性能瓶颈。")
    print("-"*50)

if __name__ == "__main__":
    run_headless_benchmark()