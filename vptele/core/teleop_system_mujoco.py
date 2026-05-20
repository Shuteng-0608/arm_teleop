import time
import numpy as np
from utils.logger import get_logger
logger = get_logger()

class TeleopSystemMujoco:
    """遥操控系统主类，协调所有模块工作"""
    
    def __init__(self, config):
        """
        初始化遥操控系统
        
        参数:
            config (dict): 系统配置
        """
        self.config = config
        self.vp_streamer = None
        self.robot_controller = None
        self.arm_teleop = None
        self.end_effector = None
        self.gripper_controller = None
        

    def initialize(self, mode="full"):
        """
        初始化系统各组件
        """
        self._initialize_full_mode()
    

    def _initialize_full_mode(self):
        """完整初始化所有组件（用于遥操控）"""
        # 初始化VisionPro数据流
        logger.info("正在初始化VisionPro数据流...")
       
        from core.vp_streamer_avp import VPStreamer
        vp_ip = self.config.get("vp_ip")
        vp_record = self.config.get("vp_record", False)

        if not vp_ip:
            raise ValueError("配置中缺少 vp_ip")

        self.vp_streamer = VPStreamer(ip=vp_ip, record=vp_record)
        # 等待一段时间确保数据流稳定
        logger.info("正在等待VisionPro数据流稳定...")
        self._wait_for_vp_ready(timeout=2.0)
        logger.info("VisionPro数据流初始化完成")

        # 初始化机械臂控制器和其他组件
        self._initialize_robot_controller()
        
        # 初始化机械臂遥控
        logger.info("正在初始化机械臂遥控模块...")

        from arm_control.arm_teleop_mujoco import ArmTeleopMujoco
        self.arm_teleop = ArmTeleopMujoco(
            self.vp_streamer,
            self.robot_controller,
            self.config.get('arm_config', {})
        )

        # 初始化末端执行器
        # peg-tool 模型默认不启用灵巧手末端执行器
        enable_hand = self.config.get("enable_hand", False)

        if enable_hand:
            self._initialize_end_effector()
        else:
            self.end_effector = None
            logger.info("已禁用手部遥操作模块，当前使用 peg-tool 模型。")
                
        logger.info("系统完整初始化完成")
    
    def _wait_for_vp_ready(self, timeout=10.0):
        start = time.time()

        while time.time() - start < timeout:
            data = self.vp_streamer.latest

            if isinstance(data, dict):
                right_wrist = self.vp_streamer.get_hand_position("right")
                if right_wrist is not None:
                    wrist = np.asarray(right_wrist)
                    if wrist.shape == (4, 4) or (
                        wrist.ndim == 3 and wrist.shape[1:] == (4, 4) and wrist.shape[0] > 0
                    ):
                        logger.info("VisionPro right_wrist 数据已就绪")
                        return

            time.sleep(0.05)

        raise TimeoutError("VisionPro 数据流超时：未收到有效 right_wrist")
    
    def _initialize_robot_controller(self):
        """初始化 MuJoCo 机械臂控制器: peg-tool 版本"""
        logger.info("正在初始化 MuJoCo 机械臂控制器...")

        # peg-tool 模型只包含 7 个机械臂关节，不再包含灵巧手关节
        sim_config = {
            "control_mode": "qpos",
            "control_rate": 100,
            "initial_arm_joints": [-0.046, -0.2, 0.0, 1.6, -1.32, 0.005, 0.005],
            "arm_joint_names": [
                "joint_1",
                "joint_2",
                "joint_3",
                "joint_4",
                "joint_5",
                "joint_6",
                "joint_7",
            ],
        }
        model_path = self.config.get(
            "mujoco_model_path",
            "/home/stw/pangu/src/arm_teleop/model/right_arm_peg_tool.xml"
        )

        logger.info(f"初始化 MuJoCo 模型: {model_path}")

        try:
            from arm_control.robot_controller_mujoco_peg_tool import RobotControllerMuJoCoPegTool

            self.robot_controller = RobotControllerMuJoCoPegTool(
                model_path=model_path,
                config=sim_config
            )

            logger.info("MuJoCo 机械臂控制器初始化完成")

        except ImportError as e:
            logger.error("错误: 无法导入 RobotControllerMuJoCoPegTool, 请检查 PYTHONPATH 和 mujoco 是否安装正确")
            logger.error(str(e))
            raise

        except Exception as e:
            logger.error(f"错误: MuJoCo 机械臂控制器初始化失败: {e}")
            raise
    
    
    def _initialize_end_effector(self):
        """初始化末端执行器"""
        
        logger.info("正在初始化灵巧手末端执行器...")
        from end_effectors.hand.hand_teleop_mujoco import HandTeleopMujoco
        self.end_effector = HandTeleopMujoco(
            self.vp_streamer,
            self.robot_controller,
            self.config.get('hand_config', {})
        )
        

    
    def start(self):
        """启动遥操控系统"""
        # 先启动机械臂遥控
        self.arm_teleop.start()
        # self.arm_teleop.multi_start()
        
        # 如果配置了末端执行器，启动对应的遥控
        if self.end_effector:
            self.end_effector.start()
        enable_hand = self.config.get("enable_hand", False)

        if enable_hand:
            self.end_effector.start()
        else:
            logger.info("已禁用手部遥操作模块，当前使用 peg-tool 模型。")

        logger.info("遥操控系统已启动")
            

    def stop(self):
        """停止遥操控系统"""
        logger.info("正在关闭遥操控系统...")
        
        # 先停止末端执行器
        if self.end_effector:
            self.end_effector.stop()
        
        # 再停止机械臂遥控
        if self.arm_teleop:
            self.arm_teleop.stop()
        
        # 最后断开机械臂连接
        if self.robot_controller:
            self.robot_controller.disconnect()
    


