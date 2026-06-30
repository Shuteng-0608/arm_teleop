import numpy as np
import mujoco
import mujoco.viewer
import time
from typing import List, Optional, Dict
import threading

class RobotControllerMuJoCo:
    def __init__(self, model_path: str, config: Optional[dict] = None):
        self.config = config or {}
        self.lock = threading.Lock()
        self.running = True
        self.viz_thread = None  # ✅ 新增：可视化线程对象

        try:
            self.model = mujoco.MjModel.from_xml_path(model_path)
            self.data = mujoco.MjData(self.model)
            print(f"成功加载模型: {model_path}")
        except Exception as e:
            print(f"加载模型失败: {e}")
            raise RuntimeError(f"无法加载模型: {model_path}")

        self.joint_names = self._get_joint_names()
        print(f"模型包含 {len(self.joint_names)} 个关节: {self.joint_names}")

        self.target_joints = [0.0] * 19 
        print(f"已初始化 {len(self.target_joints)} 个关节目标位置")

        self.sign_multiplier = np.ones(19)
        self.sign_multiplier[5:12] = np.array([1.0, 1.0, 1.0, -1.0, 1.0, -1.0, -1.0])
        self.sign_multiplier[12:19] = np.array([1.0, -1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
        print("✅ 双臂独立关节方向修正乘数已加载！")

        # 物理状态初始化
        self.data.qpos[:len(self.target_joints)] = self.target_joints
        self.data.qvel[:] = 0.0
        mujoco.mj_forward(self.model, self.data)
        print("⚡ 机械臂已瞬间完成物理状态初始化！")

        # ✅ 仅新增：viewer 对象
        self.viewer = None
        self.start_visualization_thread()

    def _get_joint_names(self) -> List[str]:
        return [
            mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, i)
            for i in range(self.model.njnt) if mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, i)
        ]

    def set_joint_positions(self, positions: List[float]):
        if len(positions) != len(self.sign_multiplier):
            print(f"警告：传入的角度数量不匹配！")
            return
        with self.lock:
            self.target_joints = (np.array(positions) * self.sign_multiplier).tolist()

    def set_dual_arm_positions(self, right_arm_cmd: List[float], left_arm_cmd: List[float]):
        if len(right_arm_cmd) != 7 or len(left_arm_cmd) != 7:
            print(f"❌ 错误: 机械臂命令维度不匹配！")
            return
        with self.lock:
            self.target_joints[5:12] = [v * k for v, k in zip(right_arm_cmd, self.sign_multiplier[5:12])]
            self.target_joints[12:19] = [v * k for v, k in zip(left_arm_cmd, self.sign_multiplier[12:19])]

    def control_callback(self, model, data):
        with self.lock:
            data.ctrl[:len(self.target_joints)] = self.target_joints

    def step_simulation(self):
        mujoco.mj_step(self.model, self.data)
        self.control_callback(self.model, self.data)

    # ✅ 启动可视化窗口
    def launch_viewer(self):
        if self.viewer is None:
            self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
            print("✅ 可视化窗口已启动")

    # ✅ 推进一帧仿真并刷新画面
    def step_and_render(self):
        if self.viewer is None or not self.viewer.is_running():
            return
        self.step_simulation()
        self.viewer.sync()

    # ✅ 新增：启动可视化线程（核心功能）
    def start_visualization_thread(self):
        """启动可视化线程，持续推进仿真"""
        if self.viz_thread is not None and self.viz_thread.is_alive():
            print("⚠️ 可视化线程已在运行")
            return
            
        self.running = True
        self.viz_thread = threading.Thread(
            target=self._visualization_loop,
            daemon=True  # 设置为守护线程，主程序退出时自动结束
        )
        self.viz_thread.start()
        print("🚀 可视化线程已启动")

    # ✅ 新增：可视化循环（线程内运行）
    def _visualization_loop(self):
        """可视化线程的主循环"""
        # 确保 viewer 已启动
        if self.viewer is None:
            self.launch_viewer()
        
        print("🎨 可视化线程开始运行...")
        
        # 等待 viewer 完全启动
        time.sleep(0.1)
        
        while self.running and self.viewer.is_running():
            self.step_and_render()
            time.sleep(0.01)  # 约100Hz的仿真频率
        
        print("🛑 可视化线程已停止")

    # ✅ 新增：停止可视化线程
    def stop_visualization_thread(self):
        """停止可视化线程"""
        self.running = False
        if self.viz_thread is not None:
            self.viz_thread.join(timeout=1.0)
            self.viz_thread = None
        print("🛑 可视化线程已停止")

    # ✅ 新增：关闭资源
    def close(self):
        """关闭所有资源"""
        self.stop_visualization_thread()
        if self.viewer is not None:
            self.viewer.close()
            self.viewer = None
        print("🔒 资源已释放")


if __name__ == "__main__":
    config = {}
    try:
        model_path = "/home/pangu/pangu/src/arm_teleop/model/pangu3/pangu3_mujoco.xml"
        simulator = RobotControllerMuJoCo(model_path, config)

    #     # 启动可视化窗口
    #     simulator.launch_viewer()

    #     time.sleep(2)

    #     print("正在发送双臂联合测试指令...")
    #     q_ik_R = np.array([-0.046, -0.2, 0, 1.6, -1.32, 0.005, 0.005])
    #     q_ik_L = np.copy(q_ik_R)
    #     simulator.set_dual_arm_positions(q_ik_R.tolist(), q_ik_L.tolist())

    #     # 方式1：使用内置的可视化线程（推荐）
    #     print("使用可视化线程模式...")
    #     simulator.start_visualization_thread()
        
    #     # 主线程可以做其他事情，比如接收外部控制指令
    #     print("主线程继续运行，可以按Ctrl+C退出...")
    #     try:
    #         while True:
    #             # 这里可以添加其他控制逻辑，比如从网络接收指令
    #             time.sleep(1.0)
                
    #             # 示例：动态改变目标位置
    #             if int(time.time()) % 5 == 0:  # 每5秒改变一次
    #                 new_pos = q_ik_R + np.random.uniform(-0.1, 0.1, size=7)
    #                 simulator.set_dual_arm_positions(new_pos.tolist(), new_pos.tolist())
                    
    #     except KeyboardInterrupt:
    #         print("\n收到中断信号，正在退出...")
            
    #     # 方式2：手动控制仿真循环（备选）
    #     # print("使用手动循环模式...")
    #     # while simulator.viewer.is_running():
    #     #     simulator.step_and_render()
    #     #     time.sleep(0.01)

    except Exception as e:
        print(f"发生错误: {e}")
    finally:
        # 确保资源被正确释放
        if 'simulator' in locals():
            simulator.close()