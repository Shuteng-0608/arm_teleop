import time
import numpy as np
import rospy
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
        else:
            logger.info(f"使用的 VisionPro IP 地址: {vp_ip}")
            rospy.loginfo(f"使用的 VisionPro IP 地址: {vp_ip}")

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
        """初始化 MuJoCo 机械臂控制器: peg-tool contact 版本"""
        logger.info("正在初始化 MuJoCo 机械臂控制器...")

        # contact 版本：使用 actuator position control，而不是 qpos 直接写入
        sim_config = {
            "cctv_camera": self.config.get("cctv_camera", "cctv_cam"),
            # 关键：真实接触仿真必须用 actuator，而不是 qpos
            "control_mode": "actuator",

            # 是否打开 MuJoCo viewer
            "launch_viewer": True,

            # 构造 controller 后自动启动仿真线程
            "auto_start": True,

            # viewer 刷新频率，不等于物理仿真频率
            # 物理仿真频率由 XML 里的 timestep 决定
            "viewer_rate": 60,

            # 尽量按真实时间运行仿真
            "realtime": True,

            # 关节目标速度限制，单位 rad/s
            # 遥操作阶段建议先保守一点，避免接触时一帧顶太猛
            "max_joint_velocity": 0.1,

            # 初始机械臂姿态，仍然用你现在这组
            # "initial_arm_joints": [-0.046, -0.2, 0.0, 1.6, -1.32, 0.005, 0.005],
            "initial_arm_joints": self.config.get(
                "initial_arm_joints",
                [-0.046, -0.2, 0.0, 1.6, -1.32, 0.005, 0.005],
            ),

            # 机械臂关节名称
            # 注意：我的 contact controller 里读取的是 arm_joints，不是 arm_joint_names
            "arm_joints": [
                "joint_1",
                "joint_2",
                "joint_3",
                "joint_4",
                "joint_5",
                "joint_6",
                "joint_7",
            ],

            # 保持你原来 controller 的符号修正逻辑
            "arm_sign": [-1, 1, 1, -1, 1, 1, 1],

            # 等 viewer 启动的时间
            "viewer_start_wait": 1.0,

            "enable_visual_guides": self.config.get("enable_visual_guides", False),
            # 当前 wall-parallel 版本默认插入方向沿 y 轴。
            "hole_axis_world": [0.0, 1.0, 0.0],

            # 洞口可视入口点相对 hole_center 的偏移
            "hole_entrance_offset": 0.026,

            # 黄色插入方向箭头长度
            "hole_axis_arrow_length": 0.20,

            # peg->hole 引导箭头粗细
            "guide_arrow_width": 0.002,

            # 对准误差颜色阈值，单位 m
            "guide_green_threshold": 0.010,
            "guide_yellow_threshold": 0.020,

            # data recording
            "record_data": self.config.get("record_data", True),
            "record_dir": self.config.get(
                "record_dir",
                "/home/stw/pangu/src/arm_teleop/data/peg_in_hole"
            ),
            "record_force_hz": self.config.get("record_force_hz", 500.0),
            "record_state_hz": self.config.get("record_state_hz", 30.0),
            "record_all_500hz": self.config.get("record_all_500hz", True),

            # video stream config
            "show_camera_streams": True,
            "camera_stream_width": 640,
            "camera_stream_height": 480,
            "camera_stream_fps": 15.0,

            "monitor_camera_names": ["ee_cam", "base_top_cam"],

            # HDF5 recording
            "record_hdf5": self.config.get("record_hdf5", True),
            "hdf5_record_dir": self.config.get(
                "hdf5_record_dir",
                "/home/stw/pangu/src/arm_teleop/data/peg_in_hole_hdf5"
            ),
            "hdf5_force_hz": self.config.get("hdf5_force_hz", 500.0),
            "hdf5_state_hz": self.config.get("hdf5_state_hz", 30.0),
            "hdf5_image_hz": self.config.get("hdf5_image_hz", 30.0),

            # 图像第一版建议外部 JPG，HDF5 保存路径和时间戳
            "hdf5_record_images": self.config.get("hdf5_record_images", True),
            "hdf5_camera_names": self.config.get(
                "hdf5_camera_names",
                ["ee_cam", "base_top_cam"]
            ),
            "hdf5_image_width": self.config.get("hdf5_image_width", 640),
            "hdf5_image_height": self.config.get("hdf5_image_height", 480),
            "hdf5_image_format": self.config.get("hdf5_image_format", "jpg"),
            "hdf5_jpg_quality": self.config.get("hdf5_jpg_quality", 90),

            # 先自动记录，后面再升级成按键 start/stop
            "hdf5_auto_start": self.config.get("hdf5_auto_start", False),
            "hdf5_episode_label": self.config.get("hdf5_episode_label", "teleop"),
            "hdf5_max_buffer_rows": self.config.get("hdf5_max_buffer_rows", 500000),

            # ROS recording service
            "enable_recording_service": self.config.get("enable_recording_service", True),
            "recording_service_name": self.config.get("recording_service_name", "/mujoco_hdf5_recording/set_recording"),

            # Hole randomization
            "enable_hole_randomization": self.config.get("enable_hole_randomization", False),
            "randomize_hole_on_record_start": self.config.get(
                "randomize_hole_on_record_start", True
            ),
            "hole_random_body_name": self.config.get("hole_random_body_name", "wall_task"),
            "hole_random_x_range": self.config.get("hole_random_x_range", [-0.01, 0.01]),
            "hole_random_y_range": self.config.get("hole_random_y_range", [0.0, 0.0]),
            "hole_random_z_range": self.config.get("hole_random_z_range", [-0.01, 0.01]),
            "hole_random_seed": self.config.get("hole_random_seed", None),

            # Reset arm before each recording episode
            "reset_arm_on_record_start": self.config.get("reset_arm_on_record_start", True),
            "reset_ignore_teleop_duration": self.config.get("reset_ignore_teleop_duration", 0.5), 

            # Joint torque quality alarm
            "enable_joint_torque_alarm": self.config.get(
                "enable_joint_torque_alarm",
                True,
            ),
            "joint_torque_alarm_only_when_recording": self.config.get(
                "joint_torque_alarm_only_when_recording",
                True,
            ),
            "joint_torque_alarm_latched": self.config.get(
                "joint_torque_alarm_latched",
                True,
            ),
            "joint_torque_limits": self.config.get(
                "joint_torque_limits",
                20.0,
            ),
            "joint_torque_alarm_peg_rgba": self.config.get(
                "joint_torque_alarm_peg_rgba",
                [0.0, 1.0, 0.0, 1.0],
            ),
            "reset_joint_torque_alarm_on_record_start": self.config.get(
                "reset_joint_torque_alarm_on_record_start",
                True,
            ),   

            # Task success auto-stop by task sites
            "enable_task_success_auto_stop": self.config.get(
                "enable_task_success_auto_stop",
                True,
            ),
            "task_success_peg_site_name": self.config.get(
                "task_success_peg_site_name",
                "peg_tip_site",
            ),
            "task_success_hole_site_name": self.config.get(
                "task_success_hole_site_name",
                "hole_goal_site",
            ),
            "task_success_distance": self.config.get(
                "task_success_distance",
                0.006,
            ),
            "task_success_dwell_time": self.config.get(
                "task_success_dwell_time",
                0.15,
            ),
            "task_success_stop_accepting_teleop": self.config.get(
                "task_success_stop_accepting_teleop",
                True,
            ),
            "task_success_terminal_hold_time": self.config.get(
                "task_success_terminal_hold_time",
                1.0,
            ),
            "task_success_blend_to_qpos_time": self.config.get(
                "task_success_blend_to_qpos_time",
                0.2,
            ),
            "task_success_only_when_recording": self.config.get(
                "task_success_only_when_recording",
                True,
            ),
            "task_success_pending_manual_review": self.config.get(
                "task_success_pending_manual_review",
                True,
            ), 

            "enable_ft_wrench_alarm": self.config.get("enable_ft_wrench_alarm", True),
            "ft_wrench_alarm_only_when_recording": self.config.get("ft_wrench_alarm_only_when_recording", True),
            "ft_wrench_alarm_latched": self.config.get("ft_wrench_alarm_latched", True),
            "reset_ft_wrench_alarm_on_record_start": self.config.get("reset_ft_wrench_alarm_on_record_start", True),
            "ft_force_norm_limit": self.config.get("ft_force_norm_limit", 40.0),
            "ft_torque_norm_limit": self.config.get("ft_torque_norm_limit", 3.0),
            "ft_wrench_alarm_dwell_time": self.config.get("ft_wrench_alarm_dwell_time", 0.15),
            "ft_wrench_alarm_peg_rgba": self.config.get("ft_wrench_alarm_peg_rgba", [0.0, 0.2, 1.0, 1.0]),
            "both_alarm_peg_rgba": self.config.get("both_alarm_peg_rgba", [0.8, 0.0, 1.0, 1.0]),
            "ft_wrench_alarm_freeze_on_trigger": self.config.get("ft_wrench_alarm_freeze_on_trigger", False),       


        }

        model_path = self.config.get(
            "mujoco_model_path",
            "/home/stw/pangu/src/arm_teleop/model/pangu_all_right.xml"
        )

        logger.info(f"初始化 MuJoCo 模型: {model_path}")

        try:
            from arm_control.robot_controller_mujoco_peg_tool_contact import RobotControllerMuJoCoPegTool

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
        

    
    # def start(self):
    #     """启动遥操控系统"""
    #     # 先启动机械臂遥控
    #     self.arm_teleop.start()
    #     # self.arm_teleop.multi_start()
        
    #     # 如果配置了末端执行器，启动对应的遥控
    #     if self.end_effector:
    #         self.end_effector.start()
    #     enable_hand = self.config.get("enable_hand", False)

    #     if enable_hand:
    #         self.end_effector.start()
    #     else:
    #         logger.info("已禁用手部遥操作模块，当前使用 peg-tool 模型。")

    #     logger.info("遥操控系统已启动")
    
    def start(self):
        """启动遥控系统"""

        if self.config.get("teleop_controlled_by_recording", True):
            logger.info(
                "teleop_controlled_by_recording=True: "
                "系统启动后不立即启动 arm_teleop，等待 recording service 开始 episode。"
            )
        else:
            if self.arm_teleop:
                self.arm_teleop.start()

            if self.end_effector:
                self.end_effector.start()

        logger.info("遥控系统已启动")
            

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
    


