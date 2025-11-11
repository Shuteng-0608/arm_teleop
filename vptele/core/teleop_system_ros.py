import time
from utils.logger import get_logger
logger = get_logger()

class TeleopSystemROS:
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
        vp_record = self.config.get('vp_record', False)
        use_vuer = self.config.get('vuer', True)
        if use_vuer:
            from core.vp_streamer_vuer import VPStreamer
        else:
            from core.vp_streamer_avp import VPStreamer
            
        self.vp_streamer = VPStreamer(self.config['vp_ip'], record=vp_record)
        # 等待一段时间确保数据流稳定
        time.sleep(1)
        
        # 初始化机械臂控制器和其他组件
        self._initialize_robot_controller()
        
        # 初始化机械臂遥控
        logger.info("正在初始化机械臂遥控模块...")

        from arm_control.arm_teleop_ros import ArmTeleopROS
        self.arm_teleop = ArmTeleopROS(
            self.vp_streamer,
            self.robot_controller,
            self.config.get('arm_config', {})
        )

        # 初始化末端执行器
        self._initialize_end_effector() # TODO: 灵巧手控制器
        
        logger.info("系统完整初始化完成")
    

    def _initialize_robot_controller(self):
        """初始化机械臂控制器"""
        logger.info("正在初始化机械臂控制器...")
        # 配置参数
        sim_config = {
            'kp': 2000.0,
            'kd': 50.0,
            'max_force': 2000.0,
            'control_rate': 100,
            'gripper_joints': [
                "joint_thumb_1", "joint_thumb_2",
                "joint_index_1", "joint_index_2",
                "joint_middle_1", "joint_middle_2",
                "joint_ring_1", "joint_ring_2",
                "joint_little_1", "joint_little_2"
            ]
        }
        # from arm_control.robot_controller_mujoco import RobotControllerMuJoCo
        # self.robot_controller = RobotControllerMuJoCo(
        #     model_path=self.config.get('mujoco_model_path', '/home/pangu/pangu/src/arm_teleop/model/right_arm_stable.xml'),
        #     config = sim_config
        # )
    
    
    def _initialize_end_effector(self):
        """初始化末端执行器"""
        
        logger.info("正在初始化灵巧手末端执行器...")
        from end_effectors.hand.hand_teleop_ros import HandTeleopROS
        self.end_effector = HandTeleopROS(
            self.vp_streamer,
            self.robot_controller,
            self.config.get('hand_config', {})
        )
        

    
    def start(self):
        """启动遥操控系统"""
        # 先启动机械臂遥控
        # self.arm_teleop.start()
        self.arm_teleop.multi_start()
        
        # 如果配置了末端执行器，启动对应的遥控
        if self.end_effector:
            self.end_effector.start()
            
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
    


