#!/usr/bin/env python3
import time
import math
import numpy as np
import mujoco
import mujoco.viewer

def run_pure_benchmark():
    # 1. 显式定义你的模型路径
    model_path = "/home/hmj/pangu/src/arm_teleop/model/pangu_all_right.xml"
    print(f"正在加载模型: {model_path}")
    
    try:
        model = mujoco.MjModel.from_xml_path(model_path)
        data = mujoco.MjData(model)
        print("模型加载成功！")
    except Exception as e:
        print(f"模型加载失败，请检查路径: {e}")
        return

    # 2. 获取物理步长和执行器信息
    sim_timestep = model.opt.timestep  # XML 里是 0.001s (1000Hz)
    print(f"MuJoCo 物理步长 (timestep): {sim_timestep}s ({1/sim_timestep:.0f} Hz)")
    print(f"模型包含执行器数量: {model.nafter if hasattr(model, 'nafter') else model.nu}")

    # 初始关节位置 (与你的 config 一致)
    q0 = np.array([-0.046, -0.2, 0.0, 1.6, -1.32, 0.005, 0.005])
    
    # 将初始位置写入前7个关节
    for i in range(7):
        joint_name = f"joint_{i+1}"
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
        if joint_id != -1:
            qpos_addr = model.jnt_qposadr[joint_id]
            data.qpos[qpos_addr] = q0[i]
            
    mujoco.mj_forward(model, data)
    print("机械臂已初始化到默认姿态。")

    # 3. 启动 Passive Viewer 可视化窗口
    print("\n[提示] 正在启动 MuJoCo Viewer 窗口...")
    print("[提示] 请观察画面中的机械臂是否在平滑地做正弦摆动。")
    print("[提示] 关注终端打印的真实物理帧率（应该接近 1000 Hz）。")
    print("按下 ESC 或关闭窗口退出测试。\n" + "-"*50)

    # 计数与性能统计变量
    step_count = 0
    fps_start_time = time.perf_counter()
    last_fps_report = time.perf_counter()
    
    # 模拟运行总时间
    run_simulation = True

    with mujoco.viewer.launch_passive(model, data) as viewer:
        # 将视角锁定在你的 cctv_cam 上
        cam_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, "cctv_cam")
        if cam_id != -1:
            viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
            viewer.cam.fixedcamid = cam_id

        # 主仿真循环
        while viewer.is_running() and run_simulation:
            step_start = time.perf_counter()

            # --- 纯控制逻辑：给 7 个 position 执行器发送正弦波命令 ---
            t_sim = data.time
            # 构造一个 0.5Hz 的平滑正弦扰动
            target_command = q0 + 0.15 * math.sin(2 * math.pi * 0.5 * t_sim)
            
            # 写入执行器 (motor_joint_1 到 motor_joint_7)
            for i in range(7):
                joint_name = f"joint_{i+1}"
                actuator_name = f"motor_{joint_name}"
                actuator_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_name)
                if actuator_id != -1:
                    data.ctrl[actuator_id] = target_command[i]

            # 推进物理步长 (1000Hz)
            mujoco.mj_step(model, data)
            step_count += 1

            # 每 1秒 刷新一次被动渲染界面
            now = time.perf_counter()
            if now - last_fps_report >= 0.016:  # 约 60Hz 刷新一次画布
                viewer.sync()
                
            # 每 2秒 在终端打印一次实际的物理线程运行频率
            if now - last_fps_report >= 2.0:
                actual_hz = step_count / (now - fps_start_time)
                print(f"实时状态 -> 已仿真时间: {t_sim:.2f}s | 实际物理线程频率: {actual_hz:.2f} Hz")
                # 理想情况下应该非常接近 1000 Hz，如果大幅低于 1000 Hz，说明 GPU/CPU 本身就带不动物理
                step_count = 0
                fps_start_time = time.perf_counter()
                last_fps_report = now

            # 严格保持真实时间对齐 (Realtime sync)
            elapsed = time.perf_counter() - step_start
            sleep_time = sim_timestep - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

if __name__ == "__main__":
    run_pure_benchmark()