import time
import numpy as np
import rospy
from utils.logger import get_logger
from utils.mujoco_config import build_arm_teleop_config, build_controller_config
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
        if mode != "full":
            raise ValueError(
                f"TeleopSystemMujoco only supports mode='full', got {mode!r}"
            )
        self._initialize_full_mode()
    

    def _initialize_full_mode(self):
        """完整初始化所有组件（用于遥操控）"""
        # 初始化VisionPro数据流（脚本模式可跳过）
        vp_enabled = bool(self.config.get("vp_enabled", True))
        if vp_enabled:
            self.vp_streamer = self._init_visionpro()
        else:
            logger.info("vp_enabled=false，跳过 VisionPro 初始化")
            self.vp_streamer = None

        # 初始化机械臂控制器和其他组件
        self._initialize_robot_controller()

        if self.vp_streamer is not None:
            self._initialize_visionpro_video()

            # 初始化机械臂遥控
            logger.info("正在初始化机械臂遥控模块...")
            from arm_control.arm_teleop_mujoco import ArmTeleopMujoco
            arm_config = build_arm_teleop_config(self.config)
            self.arm_teleop = ArmTeleopMujoco(
                self.vp_streamer,
                self.robot_controller,
                arm_config,
            )

            # 初始化末端执行器
            default_end_effector = (
                "hand" if self.config.get("enable_hand", False) else "peg"
            )
            end_effector = str(
                self.config.get("end_effector", default_end_effector)
            ).lower()

            if end_effector == "hand":
                self._initialize_end_effector()
            else:
                self.end_effector = None
                logger.info(
                    "未启用手部遥操作模块，当前末端执行器: %s",
                    end_effector,
                )
        else:
            self.arm_teleop = None
            self.end_effector = None
            logger.info("跳过 ArmTeleop 初始化（无 VisionPro）")

        # ArmTeleop services must exist before the recording service is
        # exposed and before an auto-start episode can begin.
        self.robot_controller.activate_runtime()

        # Optionally initialise the scripted peg-in-hole controller.
        self._initialize_scripted_controller()

        logger.info("系统完整初始化完成")

    def _init_visionpro(self):
        """Initialise VisionPro streamer. Returns None on failure."""
        from core.vp_streamer_avp import VPStreamer
        vp_ip = self.config.get("vp_ip")

        if not vp_ip:
            logger.warning("配置中缺少 vp_ip，跳过 VisionPro")
            return None

        logger.info(f"使用的 VisionPro IP 地址: {vp_ip}")
        rospy.loginfo(f"使用的 VisionPro IP 地址: {vp_ip}")

        try:
            streamer = VPStreamer(ip=vp_ip, record=self.config.get("vp_record", False))
        except Exception as exc:
            logger.warning(f"VisionPro 连接失败: {exc}，继续无 VisionPro 模式")
            return None

        # 等待一段时间确保数据流稳定
        logger.info("正在等待VisionPro数据流稳定...")
        try:
            self._wait_for_vp_ready(
                timeout=float(self.config.get("vp_ready_timeout", 2.0))
            )
        except TimeoutError as exc:
            logger.warning(f"VisionPro 数据等待超时: {exc}，继续无 VisionPro 模式")
            try:
                streamer.close()
            except Exception:
                pass
            return None

        logger.info("VisionPro数据流初始化完成")
        return streamer

    def _initialize_visionpro_video(self):
        """Optionally route the final CCTV+HUD frame back to Vision Pro."""
        if not bool(self.config.get("visionpro_video_enabled", False)):
            return

        if not self.robot_controller.set_cctv_frame_sink(
            self.vp_streamer.update_video_frame
        ):
            logger.error(
                "VisionPro视频回传未启动：MuJoCo中未找到配置的CCTV相机；"
                "原有遥操作和数据记录继续运行"
            )
            return

        try:
            started = self.vp_streamer.start_video_stream(
                width=self.config.get("cctv_window_width", 1280),
                height=self.config.get("cctv_window_height", 720),
                fps=self.config.get("camera_stream_fps", 15.0),
                port=self.config.get("visionpro_video_port", 9999),
            )
        except Exception as exc:
            logger.exception(f"VisionPro视频回传初始化失败: {exc}")
            started = False

        if not started:
            # Restore the original rendering path when WebRTC cannot start.
            self.robot_controller.set_cctv_frame_sink(None)
            logger.error("VisionPro视频回传不可用，原有遥操作和数据记录继续运行")
    
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
        # Start with the complete root config so a newly-added controller
        # setting cannot be silently dropped by this adapter layer.
        sim_config = build_controller_config(self.config)
        sim_config.update({
            "cctv_camera": self.config.get("cctv_camera", "cctv_cam"),
            # 关键：真实接触仿真必须用 actuator，而不是 qpos
            "control_mode": self.config.get("control_mode", "actuator"),

            # 是否打开 MuJoCo viewer
            "launch_viewer": self.config.get("launch_viewer", True),

            # 构造 controller 后自动启动仿真线程
            "auto_start": self.config.get("auto_start", True),

            # viewer 刷新频率，不等于物理仿真频率
            # 物理仿真频率由 XML 里的 timestep 决定
            "viewer_rate": self.config.get("viewer_rate", 60.0),

            # 尽量按真实时间运行仿真
            "realtime": self.config.get("realtime", True),

            # 关节目标速度限制，单位 rad/s
            # 遥操作阶段建议先保守一点，避免接触时一帧顶太猛
            "max_joint_velocity": self.config.get("max_joint_velocity", 0.1),

            # Force-aware joint command velocity scaling / hold
            "enable_force_velocity_scaling": self.config.get(
                "enable_force_velocity_scaling",
                False,
            ),
            "force_velocity_medium_threshold": self.config.get(
                "force_velocity_medium_threshold",
                40.0,
            ),
            "force_velocity_high_threshold": self.config.get(
                "force_velocity_high_threshold",
                80.0,
            ),
            "force_velocity_medium_scale": self.config.get(
                "force_velocity_medium_scale",
                0.4,
            ),
            "force_velocity_high_scale": self.config.get(
                "force_velocity_high_scale",
                0.15,
            ),
            "enable_high_force_hold": self.config.get(
                "enable_high_force_hold",
                False,
            ),
            "high_force_hold_threshold": self.config.get(
                "high_force_hold_threshold",
                100.0,
            ),
            "high_force_hold_dwell_time": self.config.get(
                "high_force_hold_dwell_time",
                0.15,
            ),
            "high_force_hold_release_ratio": self.config.get(
                "high_force_hold_release_ratio",
                0.8,
            ),

            # 初始机械臂姿态，仍然用你现在这组
            # "initial_arm_joints": [-0.046, -0.2, 0.0, 1.6, -1.32, 0.005, 0.005],
            "initial_arm_joints": self.config.get(
                "initial_arm_joints",
                [-0.046, -0.2, 0.0, 1.6, -1.32, 0.005, 0.005],
            ),

            # 机械臂关节名称
            # 注意：我的 contact controller 里读取的是 arm_joints，不是 arm_joint_names
            "arm_joints": self.config.get(
                "arm_joints",
                [f"joint_{index}" for index in range(1, 8)],
            ),

            # 保持你原来 controller 的符号修正逻辑
            "arm_sign": self.config.get(
                "arm_sign", [-1, 1, 1, -1, 1, 1, 1]
            ),

            # 等 viewer 启动的时间
            "viewer_start_wait": self.config.get("viewer_start_wait", 1.0),

            "enable_visual_guides": self.config.get("enable_visual_guides", False),
            # 当前 wall-parallel 版本默认插入方向沿 y 轴。
            "hole_axis_world": self.config.get(
                "hole_axis_world", [0.0, 1.0, 0.0]
            ),

            # 洞口可视入口点相对 hole_center 的偏移
            "hole_entrance_offset": self.config.get(
                "hole_entrance_offset", 0.026
            ),

            # 黄色插入方向箭头长度
            "hole_axis_arrow_length": self.config.get(
                "hole_axis_arrow_length", 0.20
            ),

            # peg->hole 引导箭头粗细
            "guide_arrow_width": self.config.get("guide_arrow_width", 0.002),

            # 对准误差颜色阈值，单位 m
            "guide_green_threshold": self.config.get(
                "guide_green_threshold", 0.010
            ),
            "guide_yellow_threshold": self.config.get(
                "guide_yellow_threshold", 0.020
            ),

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
            "show_camera_streams": self.config.get("show_camera_streams", True),
            "camera_stream_width": self.config.get("camera_stream_width", 640),
            "camera_stream_height": self.config.get("camera_stream_height", 480),
            "camera_stream_fps": self.config.get("camera_stream_fps", 15.0),

            "monitor_camera_names": self.config.get(
                "monitor_camera_names",
                ["cctv_cam", "ee_cam", "base_top_cam"],
            ),
            "separate_cctv_window": self.config.get(
                "separate_cctv_window",
                False,
            ),
            "cctv_window_name": self.config.get(
                "cctv_window_name",
                "CCTV Camera",
            ),
            "cctv_window_width": self.config.get(
                "cctv_window_width",
                1280,
            ),
            "cctv_window_height": self.config.get(
                "cctv_window_height",
                720,
            ),
            "show_cctv_in_combined_panel": self.config.get(
                "show_cctv_in_combined_panel",
                True,
            ),
            "cctv_window_preserve_aspect_ratio": self.config.get(
                "cctv_window_preserve_aspect_ratio",
                True,
            ),
            "cctv_window_fullscreen": self.config.get(
                "cctv_window_fullscreen",
                False,
            ),
            "cctv_window_fit_mode": self.config.get(
                "cctv_window_fit_mode",
                "contain",
            ),
            "cctv_window_padding_color": self.config.get(
                "cctv_window_padding_color",
                [0, 0, 0],
            ),

            # Operator-only force feedback HUD. This is drawn only on live
            # OpenCV display frames, not on HDF5 recording frames.
            "enable_force_visual_feedback": self.config.get(
                "enable_force_visual_feedback",
                False,
            ),
            "force_feedback_display_mode": self.config.get(
                "force_feedback_display_mode",
                "overlay",
            ),
            "force_feedback_overlay_cameras": self.config.get(
                "force_feedback_overlay_cameras",
                ["cctv_cam"],
            ),
            "force_feedback_low_threshold": self.config.get(
                "force_feedback_low_threshold",
                10.0,
            ),
            "force_feedback_medium_threshold": self.config.get(
                "force_feedback_medium_threshold",
                40.0,
            ),
            "force_feedback_high_threshold": self.config.get(
                "force_feedback_high_threshold",
                80.0,
            ),
            "force_feedback_excessive_threshold": self.config.get(
                "force_feedback_excessive_threshold",
                100.0,
            ),
            "force_feedback_show_numbers": self.config.get(
                "force_feedback_show_numbers",
                True,
            ),
            "force_feedback_show_axial_lateral": self.config.get(
                "force_feedback_show_axial_lateral",
                True,
            ),
            "force_feedback_show_trend": self.config.get(
                "force_feedback_show_trend",
                True,
            ),
            "force_feedback_show_contact_state": self.config.get(
                "force_feedback_show_contact_state",
                True,
            ),
            "force_feedback_show_arrow": self.config.get(
                "force_feedback_show_arrow",
                False,
            ),
            "force_feedback_smoothing_alpha": self.config.get(
                "force_feedback_smoothing_alpha",
                0.25,
            ),
            "force_feedback_window_name": self.config.get(
                "force_feedback_window_name",
                "Force Feedback HUD",
            ),
            "force_feedback_insertion_axis_world": self.config.get(
                "force_feedback_insertion_axis_world",
                [0.0, -1.0, 0.0],
            ),
            "force_feedback_use_compensated_wrench": self.config.get(
                "force_feedback_use_compensated_wrench",
                True,
            ),
            "force_feedback_wrench_label": self.config.get(
                "force_feedback_wrench_label",
                "comp",
            ),
            "force_feedback_trend_window_sec": self.config.get(
                "force_feedback_trend_window_sec",
                0.8,
            ),
            "force_feedback_trend_rising_threshold": self.config.get(
                "force_feedback_trend_rising_threshold",
                5.0,
            ),
            "force_feedback_trend_falling_threshold": self.config.get(
                "force_feedback_trend_falling_threshold",
                -5.0,
            ),
            "force_feedback_contact_free_threshold": self.config.get(
                "force_feedback_contact_free_threshold",
                5.0,
            ),
            "force_feedback_contact_light_threshold": self.config.get(
                "force_feedback_contact_light_threshold",
                15.0,
            ),
            "force_feedback_axial_high_threshold": self.config.get(
                "force_feedback_axial_high_threshold",
                20.0,
            ),
            "force_feedback_lateral_high_threshold": self.config.get(
                "force_feedback_lateral_high_threshold",
                10.0,
            ),
            "force_feedback_jam_force_threshold": self.config.get(
                "force_feedback_jam_force_threshold",
                25.0,
            ),
            "force_feedback_jam_lateral_threshold": self.config.get(
                "force_feedback_jam_lateral_threshold",
                12.0,
            ),
            "enable_task_force_guidance_hud": self.config.get(
                "enable_task_force_guidance_hud",
                True,
            ),
            "task_force_guidance_mode": self.config.get(
                "task_force_guidance_mode",
                "ring",
            ),
            "force_guidance_overlay_cameras": self.config.get(
                "force_guidance_overlay_cameras",
                ["cctv_cam"],
            ),
            "show_translation_ring": self.config.get(
                "show_translation_ring",
                True,
            ),
            "show_torque_ring": self.config.get(
                "show_torque_ring",
                True,
            ),
            "show_axial_core": self.config.get(
                "show_axial_core",
                True,
            ),
            "force_guidance_plane_right_world": self.config.get(
                "force_guidance_plane_right_world",
                [1.0, 0.0, 0.0],
            ),
            "force_guidance_plane_up_world": self.config.get(
                "force_guidance_plane_up_world",
                [0.0, 0.0, 1.0],
            ),
            "force_guidance_ring_radius_px": self.config.get(
                "force_guidance_ring_radius_px",
                70,
            ),
            "force_guidance_max_vector_px": self.config.get(
                "force_guidance_max_vector_px",
                55,
            ),
            "force_guidance_max_force_n": self.config.get(
                "force_guidance_max_force_n",
                40.0,
            ),
            "force_guidance_max_torque_nm": self.config.get(
                "force_guidance_max_torque_nm",
                2.0,
            ),
            "torque_guidance_mode": self.config.get(
                "torque_guidance_mode",
                "tilt_axes",
            ),
            "torque_guidance_min_force_n": self.config.get(
                "torque_guidance_min_force_n",
                5.0,
            ),
            "torque_guidance_min_torque_nm": self.config.get(
                "torque_guidance_min_torque_nm",
                0.05,
            ),
            "torque_guidance_max_torque_nm": self.config.get(
                "torque_guidance_max_torque_nm",
                self.config.get("force_guidance_max_torque_nm", 2.0),
            ),
            "torque_guidance_label_as_posture": self.config.get(
                "torque_guidance_label_as_posture",
                True,
            ),
            "torque_guidance_show_numeric_values": self.config.get(
                "torque_guidance_show_numeric_values",
                True,
            ),
            "force_guidance_draw_numeric_values": self.config.get(
                "force_guidance_draw_numeric_values",
                True,
            ),
            "force_guidance_show_caveat_label": self.config.get(
                "force_guidance_show_caveat_label",
                True,
            ),
            "force_guidance_hud_anchor": self.config.get(
                "force_guidance_hud_anchor",
                "top_right",
            ),
            "force_guidance_hud_margin_px": self.config.get(
                "force_guidance_hud_margin_px",
                [40, 40],
            ),
            "force_guidance_hud_offset_px": self.config.get(
                "force_guidance_hud_offset_px",
                [0, 0],
            ),
            "force_guidance_hud_center_norm": self.config.get(
                "force_guidance_hud_center_norm",
                [0.78, 0.28],
            ),
            "force_guidance_basis_mode": self.config.get(
                "force_guidance_basis_mode",
                "camera_screen",
            ),
            "force_guidance_basis_camera": self.config.get(
                "force_guidance_basis_camera",
                "cctv_cam",
            ),
            "force_guidance_screen_right_sign": self.config.get(
                "force_guidance_screen_right_sign",
                1.0,
            ),
            "force_guidance_screen_up_sign": self.config.get(
                "force_guidance_screen_up_sign",
                1.0,
            ),
            "force_guidance_vector_semantics": self.config.get(
                "force_guidance_vector_semantics",
                "contact",
            ),
            "force_guidance_correction_sign": self.config.get(
                "force_guidance_correction_sign",
                -1.0,
            ),
            "force_feedback_ft_compensation_mode": self.config.get(
                "force_feedback_ft_compensation_mode",
                self.config.get("hdf5_ft_compensation_mode", "gravity"),
            ),
            "force_feedback_gravity_tool_body_names": self.config.get(
                "force_feedback_gravity_tool_body_names",
                self.config.get("hdf5_ft_gravity_tool_body_names", ["peg_tool"]),
            ),
            "force_feedback_gravity_world": self.config.get(
                "force_feedback_gravity_world",
                self.config.get("hdf5_ft_gravity_world", [0.0, 0.0, -9.81]),
            ),
            "force_feedback_gravity_sensor_sign": self.config.get(
                "force_feedback_gravity_sensor_sign",
                self.config.get("hdf5_ft_gravity_sensor_sign", -1.0),
            ),

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
            "hdf5_hole_center_site_name": self.config.get(
                "hdf5_hole_center_site_name",
                self.config.get("task_success_hole_site_name", "hole_goal_site"),
            ),
            "hdf5_peg_geom_name": self.config.get(
                "hdf5_peg_geom_name", "cylindrical_peg"
            ),
            "hdf5_peg_tip_site_name": self.config.get(
                "hdf5_peg_tip_site_name", "peg_tip_site"
            ),
            "hdf5_hole_goal_site_name": self.config.get(
                "hdf5_hole_goal_site_name",
                self.config.get("task_success_hole_site_name", "hole_goal_site"),
            ),
            "hdf5_hole_ring_geom_prefix": self.config.get(
                "hdf5_hole_ring_geom_prefix", "wall_hole_ring_"
            ),
            "hdf5_hole_axis_body": self.config.get(
                "hdf5_hole_axis_body", [0.0, -1.0, 0.0]
            ),

            # ROS recording service
            "enable_recording_service": self.config.get("enable_recording_service", True),
            "recording_service_name": self.config.get("recording_service_name", "/mujoco_hdf5_recording/set_recording"),

            # Hole position sampling
            "hole_sampling_mode": self.config.get(
                "hole_sampling_mode",
                (
                    "uniform_random"
                    if self.config.get("enable_hole_randomization", False)
                    else "fixed"
                ),
            ),
            "hole_body_name": self.config.get(
                "hole_body_name",
                self.config.get("hole_random_body_name", "wall_task"),
            ),
            "hole_grid_rows": self.config.get("hole_grid_rows", 5),
            "hole_grid_cols": self.config.get("hole_grid_cols", 5),
            "hole_grid_x_range": self.config.get(
                "hole_grid_x_range", [-0.06, 0.06]
            ),
            "hole_grid_y_offset": self.config.get("hole_grid_y_offset", 0.0),
            "hole_grid_z_range": self.config.get(
                "hole_grid_z_range", [-0.06, 0.06]
            ),
            "hole_grid_sample_mode": self.config.get(
                "hole_grid_sample_mode", "center"
            ),
            "hole_grid_traversal_order": self.config.get(
                "hole_grid_traversal_order", "shuffled"
            ),
            "hole_grid_seed": self.config.get("hole_grid_seed", 42),
            "hole_grid_advance_policy": self.config.get(
                "hole_grid_advance_policy", "on_keep"
            ),
            "hole_grid_start_cycle": self.config.get("hole_grid_start_cycle", 0),
            "hole_grid_start_index": self.config.get("hole_grid_start_index", 0),

            # Legacy uniform-random compatibility
            "enable_hole_randomization": self.config.get("enable_hole_randomization", False),
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


        })

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

    def _initialize_scripted_controller(self):
        """Optionally initialise the scripted peg-in-hole controller."""
        sc_cfg = self.config.get("scripted_controller", {})
        if not bool(sc_cfg.get("enabled", False)):
            return

        logger.info("正在初始化脚本插入控制器...")

        # The scripted waypoint controller uses MuJoCo Jacobian IK. ROS IK is
        # optional and only kept as a fallback for older scripted controllers.
        ik_proxy = None
        try:
            ik_service_name = self.config.get("arm_config", {}).get(
                "ik_service_name", "/arm_teleop/right_arm_ik_srv"
            )
            rospy.wait_for_service(ik_service_name, timeout=3.0)
            from arm_teleop.srv import ArmIK
            ik_proxy = rospy.ServiceProxy(ik_service_name, ArmIK)
            logger.info("ROS IK 服务可用（备用）")
        except (rospy.ROSException, rospy.ServiceException):
            logger.info("ROS IK 服务不可用，使用 MuJoCo Jacobian IK")

        from arm_control.scripted_insertion_ros import ScriptedInsertionROSNode
        self.scripted_insertion_node = ScriptedInsertionROSNode(
            robot_controller=self.robot_controller,
            ik_service_proxy=ik_proxy,
            config=sc_cfg,
        )
        logger.info("脚本插入控制器初始化完成")
        

    
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
        try:
            # 先停止末端执行器
            if self.end_effector:
                self.end_effector.stop()

            # 再停止机械臂遥控
            if self.arm_teleop:
                self.arm_teleop.stop()

            # 先停止渲染线程，再释放其使用的视频发送端。
            if self.robot_controller:
                self.robot_controller.disconnect()
        finally:
            if self.vp_streamer:
                self.vp_streamer.close()
    
