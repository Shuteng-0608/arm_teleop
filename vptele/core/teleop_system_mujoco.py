import time
import numpy as np
from utils.logger import get_logger
logger = get_logger()

class TeleopSystemMujoco:
    def __init__(self, config):
        self.config = config
        self.robot_controller = None
        self._viz_thread_started = False

    def initialize(self):
        self._initialize_robot_controller()
        self._execute_initial_pose()  # 执行起手式
        self._start_visualization_thread()  # 启动可视化
        logger.info("✅ 系统初始化完成")

    def _initialize_robot_controller(self):
        logger.info("初始化MuJoCo控制器...")
        from arm_control.new_test import RobotControllerMuJoCo  # 确保导入上面的修改版
        self.robot_controller = RobotControllerMuJoCo(
            model_path="/home/pangu/pangu/src/arm_teleop/model/pangu3/pangu3_mujoco.xml",
            config={
                'kp': 5,  # 手臂执行器kp，和XML一致
                'kv': 1,
                'control_rate': 100
            }
        )
        # 打印关节顺序，确认和XML一致
        logger.info(f"模型关节顺序: {self.robot_controller.joint_names}")

    def _execute_initial_pose(self):
        """起手式：完全对齐XML初始姿态，无J3关节"""
        logger.info("执行起手式...")
        # 1. 躯干初始姿态（yaw是平移关节，单位米，给0.0）
        torso_home = [0.0, 0.0, 0.0, 0.0, 0.0]  # [yaw, pigu, body, dai, dai2]
        # 2. 右臂初始姿态（J4R~J10R，7个值）
        right_arm_home = np.array([
            -0.046,   # J4R: 肩部旋转
            -0.2,     # J5R: 肩部俯仰（前伸）
            0.0,      # J6R_Joint: 肘部俯仰
            1.6,      # J7R: 肘部旋转
            -1.32,    # J8R: 腕部俯仰
            0.005,    # J9R: 腕部旋转
            0.005     # J10R: 末端旋转
        ])
        # 3. 左臂初始姿态（J4L~J10L，7个值，和之前一致，符号由控制器自动修正）
        left_arm_home = np.array([
            -0.046,    # J4L: 肩部旋转（对称）
            -0.2,     # J5L: 肩部俯仰（前伸）
            1.6,      # J6L: 肘部俯仰（控制器会自动乘-1修正方向）
            -1.32,    # J7L: 肘部旋转
            1.32,     # J8L: 腕部俯仰
            0.005,   # J9L: 腕部旋转
            0.005    # J10L: 末端旋转（控制器会自动乘-1修正方向）
        ])
        # 4. 下发初始姿态
        self.robot_controller.set_torso_positions(torso_home)
        self.robot_controller.set_dual_arm_positions(right_arm_cmd=right_arm_home.tolist(),
                                                    left_arm_cmd=left_arm_home.tolist())
        # 5. 等待物理稳定
        for _ in range(20):
            self.robot_controller.step_simulation()
            time.sleep(0.001)
        # 6. 打印当前状态，验证正确性
        curr = self.robot_controller.get_current_joints()
        logger.info(f"起手式完成，躯干关节: {[f'{x:.3f}' for x in curr[:5]]}")
        logger.info(f"右臂关节: {[f'{x:.3f}' for x in curr[5:12]]}")
        logger.info(f"左臂关节: {[f'{x:.3f}' for x in curr[12:19]]}")

    def _start_visualization_thread(self):
        if not self._viz_thread_started:
            self.robot_controller.start_visualization_thread()
            self._viz_thread_started = True
            logger.info("✅ 可视化线程启动")

    def start(self):
        """启动遥操作（这里放你原来的VisionPro/遥操作逻辑）"""
        logger.info("🚀 遥操控系统启动，等待指令...")
        # 示例：持续打印关节状态，后续替换为你的遥操作逻辑
        while self.robot_controller.viewer.is_running():
            time.sleep(1)
            curr = self.robot_controller.get_current_joints()
            logger.debug(f"当前关节: {[f'{x:.2f}' for x in curr]}")

    def stop(self):
        self.robot_controller.close()
        logger.info("✅ 系统关闭")

if __name__ == "__main__":
    config = {}
    sys = TeleopSystemMujoco(config)
    sys.initialize()
    sys.start()