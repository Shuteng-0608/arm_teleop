#!/usr/bin/env python3
"""
MuJoCo controller for the 7-DoF right-arm peg-tool model.

Contact-oriented version:
- default control mode is actuator position control, not direct qpos overwriting;
- MuJoCo dynamics and contacts are stepped with mj_step();
- target joint commands are rate-limited before being sent to actuators;
- original public methods are preserved where possible:
    set_arm_positions(...)
    set_hand_positions(...)
    get_current_joints(...)
    update_positions(...)
    start_simulation(...)
    disconnect(...)

Important:
    In actuator mode, contacts can stop the peg instead of being overwritten
    by direct qpos assignment. This is the key change for real contact behavior.
"""

from __future__ import annotations

import copy
import queue
import time
import threading
from dataclasses import dataclass
from typing import List, Optional, Dict, Any, Callable

import numpy as np
import cv2
import mujoco
import mujoco.viewer
import shutil

try:
    from vptele.utils.mujoco_data_recorder import MujocoDataRecorder
    from vptele.utils.mujoco_hdf5_recorder import (
        ImageCaptureRequest,
        MujocoHDF5Recorder,
    )
    from vptele.utils.hole_grid_scheduler import HoleGridScheduler
    from vptele.utils.force_feedback_overlay import (
        ForceFeedbackConfig,
        ForceFeedbackSmoother,
        compute_force_feedback,
        draw_force_feedback_overlay,
        make_force_feedback_hud,
        resize_with_aspect_padding,
        trend_label,
    )
    from vptele.utils.ft_wrench_utils import (
        body_ids as ft_body_ids,
        compensated_ft_wrench,
        ft_sensor_pose_world,
        gravity_wrench_sensor_frame,
        raw_ft_wrench,
    )
except ModuleNotFoundError:  # Catkin's legacy package_dir exposes utils directly.
    from utils.mujoco_data_recorder import MujocoDataRecorder
    from utils.mujoco_hdf5_recorder import ImageCaptureRequest, MujocoHDF5Recorder
    from utils.hole_grid_scheduler import HoleGridScheduler
    from utils.force_feedback_overlay import (
        ForceFeedbackConfig,
        ForceFeedbackSmoother,
        compute_force_feedback,
        draw_force_feedback_overlay,
        make_force_feedback_hud,
        resize_with_aspect_padding,
        trend_label,
    )
    from utils.ft_wrench_utils import (
        body_ids as ft_body_ids,
        compensated_ft_wrench,
        ft_sensor_pose_world,
        gravity_wrench_sensor_frame,
        raw_ft_wrench,
    )

@dataclass
class _RenderSnapshot:
    display_requested: bool
    recording_request: Optional[ImageCaptureRequest]
    time: float
    qpos: np.ndarray
    qvel: np.ndarray
    act: np.ndarray
    ctrl: np.ndarray
    qacc_warmstart: np.ndarray
    qfrc_applied: np.ndarray
    xfrc_applied: np.ndarray
    mocap_pos: np.ndarray
    mocap_quat: np.ndarray
    userdata: np.ndarray
    sensordata: np.ndarray
    body_pos: np.ndarray
    mat_rgba: np.ndarray
    geom_rgba: np.ndarray


class RobotControllerMuJoCoPegTool:
    def __init__(self, model_path: str, config: Optional[Dict[str, Any]] = None):

        self.config = config or {}
        self.model_path = model_path



        

        


        # ########################################################### #
        # -------------------- Load MuJoCo model -------------------- #
        # ########################################################### #

        try:
            self.model = mujoco.MjModel.from_xml_path(model_path)
            self.data = mujoco.MjData(self.model)
            print(f"成功加载模型: {model_path}")
        except Exception as e:
            print(f"加载模型失败: {e}")
            raise RuntimeError(f"无法加载模型: {model_path}") from e

        self.joint_names = self._get_joint_names()
        print(f"模型包含 {len(self.joint_names)} 个关节: {self.joint_names}")

        default_arm_joints = [f"joint_{i}" for i in range(1, 8)]
        self.arm_joint_names = self.config.get("arm_joints", default_arm_joints)
        self.arm_joint_names = [j for j in self.arm_joint_names if j in self.joint_names]

        if len(self.arm_joint_names) != 7:
            print(
                f"警告: 当前识别到的机械臂关节数量为 {len(self.arm_joint_names)}，"
                f"期望为 7。arm_joint_names={self.arm_joint_names}"
            )

        self._arm_joint_dof_addresses = np.asarray(
            [
                self.model.jnt_dofadr[
                    mujoco.mj_name2id(
                        self.model,
                        mujoco.mjtObj.mjOBJ_JOINT,
                        joint_name,
                    )
                ]
                for joint_name in self.arm_joint_names
            ],
            dtype=np.int32,
        )
        self._sensor_address_cache = {}
        for sensor_name in ("peg_ft_force", "peg_ft_torque"):
            sensor_id = mujoco.mj_name2id(
                self.model,
                mujoco.mjtObj.mjOBJ_SENSOR,
                sensor_name,
            )
            self._sensor_address_cache[sensor_name] = (
                None
                if sensor_id == -1
                else (
                    int(self.model.sensor_adr[sensor_id]),
                    int(self.model.sensor_dim[sensor_id]),
                )
            )

        self.actuator_map = self._create_actuator_map()

        # Default: actuator mode for physical contact.
        self.control_mode = self.config.get("control_mode", "actuator")
        if self.control_mode not in {"actuator", "qpos"}:
            print(f"警告: 未知 control_mode={self.control_mode}，自动切换为 actuator")
            self.control_mode = "actuator"

        # Sign convention kept from your original controller.
        self.arm_sign = self.config.get("arm_sign", [-1, 1, 1, -1, 1, 1, 1])





        # ################################################################## #
        # ================== Episode-level teleop control ================== #
        # ################################################################## #
        self.teleop_controlled_by_recording = bool(
            self.config.get("teleop_controlled_by_recording", True)
        )

        self.accept_teleop_commands = bool(
            self.config.get("accept_teleop_when_not_recording", False)
        )

        self.teleop_stop_service_name = self.config.get(
            "teleop_stop_service_name",
            "/arm_teleop_mujoco/stop",
        )

        self.teleop_recalibrate_service_name = self.config.get(
            "teleop_recalibrate_service_name",
            "/arm_teleop_mujoco/recalibrate",
        )

        self.teleop_start_service_name = self.config.get(
            "teleop_start_service_name",
            "/arm_teleop_mujoco/start",
        )

        self.reset_arm_on_record_stop = bool(
            self.config.get("reset_arm_on_record_stop", True)
        )




        # ############################################################### #
        # -------------------- Camera stream monitor -------------------- #
        # ############################################################### #

        self.cctv_camera = self.config.get("cctv_camera", "cctv_cam")
        self.show_camera_streams = bool(self.config.get("show_camera_streams", True))
        self.camera_stream_width = int(self.config.get("camera_stream_width", 640))
        self.camera_stream_height = int(self.config.get("camera_stream_height", 480))
        self.camera_stream_fps = float(self.config.get("camera_stream_fps", 15.0))
        self.camera_stream_period = 1.0 / max(self.camera_stream_fps, 1.0)

        self.monitor_camera_names = self.config.get(
            "monitor_camera_names",
            ["cctv_cam", "ee_cam", "base_top_cam"]
        )

        self.separate_cctv_window = bool(
            self.config.get("separate_cctv_window", False)
        )
        self.cctv_window_name = self.config.get("cctv_window_name", "CCTV Camera")
        self.cctv_window_width = int(self.config.get("cctv_window_width", 1280))
        self.cctv_window_height = int(self.config.get("cctv_window_height", 720))
        self.show_cctv_in_combined_panel = bool(
            self.config.get("show_cctv_in_combined_panel", True)
        )
        self.cctv_window_preserve_aspect_ratio = bool(
            self.config.get("cctv_window_preserve_aspect_ratio", True)
        )
        self.cctv_window_fullscreen = bool(
            self.config.get("cctv_window_fullscreen", False)
        )
        self.cctv_window_fit_mode = self.config.get(
            "cctv_window_fit_mode",
            "contain",
        )
        if self.cctv_window_fit_mode != "contain":
            print(
                f"[Camera Monitor] Unsupported cctv_window_fit_mode="
                f"{self.cctv_window_fit_mode}. Falling back to contain."
            )
            self.cctv_window_fit_mode = "contain"
        self.cctv_window_padding_color = self.config.get(
            "cctv_window_padding_color",
            [0, 0, 0],
        )
        self.cctv_window_initialized = False
        self.cctv_frame_sink: Optional[Callable[[np.ndarray], Any]] = None
        self.cctv_frame_sink_error_count = 0
        self._last_cctv_frame_sink_error_log_time = 0.0

        self.monitor_camera_ids = {}

        if self.show_camera_streams:
            for cam_name in self.monitor_camera_names:
                cam_id = mujoco.mj_name2id(
                    self.model,
                    mujoco.mjtObj.mjOBJ_CAMERA,
                    cam_name
                )
                if cam_id == -1:
                    print(f"[Camera Monitor] Warning: camera not found: {cam_name}")
                else:
                    self.monitor_camera_ids[cam_name] = cam_id

            if len(self.monitor_camera_ids) == 0:
                print("[Camera Monitor] No valid monitor cameras found, disable stream display.")
                self.show_camera_streams = False



        # ############################################################### #
        # ---------------- Operator force feedback HUD ----------------- #
        # ############################################################### #

        self.force_feedback_config = ForceFeedbackConfig.from_dict(self.config)
        self.force_feedback_smoother = ForceFeedbackSmoother(
            alpha=self.force_feedback_config.smoothing_alpha
        )
        self.force_feedback_history = []
        self.force_feedback_raw_fallback_warned = False
        self.force_guidance_basis_fallback_warned = False

        self.force_feedback_ft_sensor_site_id = -1
        ft_force_sensor_id = mujoco.mj_name2id(
            self.model,
            mujoco.mjtObj.mjOBJ_SENSOR,
            "peg_ft_force",
        )
        ft_torque_sensor_id = mujoco.mj_name2id(
            self.model,
            mujoco.mjtObj.mjOBJ_SENSOR,
            "peg_ft_torque",
        )

        if ft_force_sensor_id != -1:
            self.force_feedback_ft_sensor_site_id = int(
                self.model.sensor_objid[ft_force_sensor_id]
            )
        elif ft_torque_sensor_id != -1:
            self.force_feedback_ft_sensor_site_id = int(
                self.model.sensor_objid[ft_torque_sensor_id]
            )

        if self.force_feedback_config.enabled:
            print("[ForceFeedbackHUD] enabled")
            print(f"  display_mode    = {self.force_feedback_config.display_mode}")
            print(f"  overlay_cameras = {self.force_feedback_config.overlay_cameras}")
            print(f"  wrench_source   = {self.force_feedback_config.wrench_label}")

        self.force_feedback_compensation_mode = self.config.get(
            "force_feedback_ft_compensation_mode",
            self.config.get("hdf5_ft_compensation_mode", "gravity"),
        )
        self.force_feedback_gravity_tool_body_names = self.config.get(
            "force_feedback_gravity_tool_body_names",
            self.config.get("hdf5_ft_gravity_tool_body_names", ["peg_tool"]),
        )
        self.force_feedback_gravity_tool_body_ids = ft_body_ids(
            self.model,
            self.force_feedback_gravity_tool_body_names,
        )
        self.force_feedback_gravity_world = self.config.get(
            "force_feedback_gravity_world",
            self.config.get("hdf5_ft_gravity_world", [0.0, 0.0, -9.81]),
        )
        self.force_feedback_gravity_sensor_sign = float(
            self.config.get(
                "force_feedback_gravity_sensor_sign",
                self.config.get("hdf5_ft_gravity_sensor_sign", -1.0),
            )
        )




        # ######################################################### #
        # -------------------- Visual guidance -------------------- #
        # ######################################################### #
        self.enable_visual_guides = bool(self.config.get("enable_visual_guides", True))

        self.hole_axis_world = np.array(
            self.config.get("hole_axis_world", [0.0, -1.0, 0.0]),
            dtype=float,
        )

        axis_norm = np.linalg.norm(self.hole_axis_world)
        if axis_norm < 1e-8:
            self.hole_axis_world = np.array([0.0, -1.0, 0.0], dtype=float)
        else:
            self.hole_axis_world = self.hole_axis_world / axis_norm

        # 洞口 marker 相对于 hole_center 的偏移。
        # 你的 hole 深度约 0.045 m，半深度 0.0225 m，所以 0.026 会稍微浮在洞口外侧。
        self.hole_entrance_offset = float(self.config.get("hole_entrance_offset", 0.026))

        # 黄色洞口法向/插入方向箭头长度
        self.hole_axis_arrow_length = float(self.config.get("hole_axis_arrow_length", 0.08))

        # peg->hole 引导箭头粗细
        self.guide_arrow_width = float(self.config.get("guide_arrow_width", 0.006))

        # 颜色切换阈值，单位 m。
        # 这里用的是横向对准误差，不是沿插入方向的距离。
        self.guide_green_threshold = float(self.config.get("guide_green_threshold", 0.010))
        self.guide_yellow_threshold = float(self.config.get("guide_yellow_threshold", 0.030))
        



        # ########################################################## #
        # -------------------- Visual peg color -------------------- #
        # ########################################################## #

        self.peg_geom_id = mujoco.mj_name2id(
            self.model,
            mujoco.mjtObj.mjOBJ_GEOM,
            "cylindrical_peg",
        )

        self.peg_mat_id = mujoco.mj_name2id(
            self.model,
            mujoco.mjtObj.mjOBJ_MATERIAL,
            "mat_peg",
        )

        self.default_peg_rgba = np.array([0.90, 0.18, 0.10, 1.0], dtype=float)

        # Simulation and visualization timing.
        self.sim_timestep = float(self.model.opt.timestep)
        self.viewer_rate = float(self.config.get("viewer_rate", 60.0))
        self.viewer_period = 1.0 / max(self.viewer_rate, 1.0)
        self.realtime = bool(self.config.get("realtime", True))



        # ######################################################## #
        # -------------------- Data recording -------------------- #
        # ######################################################## #

        self.data_recorder = None

        if bool(self.config.get("record_data", False)):
            self.data_recorder = MujocoDataRecorder(
                model=self.model,
                data=self.data,
                output_dir=self.config.get(
                    "record_dir",
                    "/home/stw/pangu/src/arm_teleop/data/peg_in_hole"
                ),
                model_path=self.model_path,
                force_hz=float(self.config.get("record_force_hz", 500.0)),
                state_hz=float(self.config.get("record_state_hz", 30.0)),
                write_all_500hz=bool(self.config.get("record_all_500hz", True)),
            )
        

        # ############################################################# #
        # -------------------- HDF5 data recording -------------------- #
        # ############################################################# #

        self.hdf5_recorder = None

        self.hdf5_auto_start = bool(self.config.get("hdf5_auto_start", False))

        if bool(self.config.get("record_hdf5", False)):
            self.hdf5_recorder = MujocoHDF5Recorder(
                model=self.model,
                data=self.data,
                output_dir=self.config.get(
                    "hdf5_record_dir",
                    "/home/stw/pangu/src/arm_teleop/data/peg_in_hole_hdf5"
                ),
                model_path=self.model_path,
                force_hz=float(self.config.get("hdf5_force_hz", 500.0)),
                state_hz=float(self.config.get("hdf5_state_hz", 30.0)),
                image_hz=float(self.config.get("hdf5_image_hz", 30.0)),
                record_images=bool(self.config.get("hdf5_record_images", True)),
                camera_names=self.config.get(
                    "hdf5_camera_names",
                    ["ee_cam", "base_top_cam"]
                ),
                image_width=int(self.config.get("hdf5_image_width", 640)),
                image_height=int(self.config.get("hdf5_image_height", 480)),
                image_format=self.config.get("hdf5_image_format", "jpg"),
                jpg_quality=int(self.config.get("hdf5_jpg_quality", 90)),
                max_buffer_rows=int(self.config.get("hdf5_max_buffer_rows", 500000)),
                async_queue_size=int(
                    self.config.get("hdf5_async_queue_size", 1024)
                ),
                append_block_image=int(
                    self.config.get("hdf5_append_block_image", 16)
                ),
                async_stop_timeout=float(
                    self.config.get("hdf5_async_stop_timeout", 10.0)
                ),

                enable_ft_tare=bool(self.config.get("hdf5_enable_ft_tare", False)),
                # record_ft_wrench_raw=bool(self.config.get("hdf5_record_ft_wrench_raw", True)),

                ft_compensation_mode=self.config.get(
                    "hdf5_ft_compensation_mode",
                    "gravity",
                ),
                record_ft_wrench_raw=bool(
                    self.config.get("hdf5_record_ft_wrench_raw", True)
                ),
                record_ft_wrench_gravity=bool(
                    self.config.get("hdf5_record_ft_wrench_gravity", True)
                ),

                record_actions=bool(
                    self.config.get("hdf5_record_actions", True)
                ),
                record_action_alias=bool(
                    self.config.get("hdf5_record_action_alias", True)
                ),

                ft_gravity_tool_body_names=self.config.get(
                    "hdf5_ft_gravity_tool_body_names",
                    ["peg_tool"],
                ),
                ft_gravity_world=self.config.get(
                    "hdf5_ft_gravity_world",
                    [0.0, 0.0, -9.81],
                ),
                ft_gravity_sensor_sign=float(
                    self.config.get("hdf5_ft_gravity_sensor_sign", -1.0)
                ),
                peg_geom_name=self.config.get(
                    "hdf5_peg_geom_name",
                    "cylindrical_peg",
                ),
                peg_tip_site_name=self.config.get(
                    "hdf5_peg_tip_site_name",
                    "peg_tip_site",
                ),
                hole_center_site_name=self.config.get(
                    "hdf5_hole_center_site_name",
                    self.config.get("task_success_hole_site_name", "hole_goal_site"),
                ),
                hole_goal_site_name=self.config.get(
                    "hdf5_hole_goal_site_name",
                    self.config.get("task_success_hole_site_name", "hole_goal_site"),
                ),
                hole_ring_geom_prefix=self.config.get(
                    "hdf5_hole_ring_geom_prefix",
                    "wall_hole_ring_",
                ),
                hole_axis_body=self.config.get(
                    "hdf5_hole_axis_body",
                    [0.0, -1.0, 0.0],
                ),
            )



        # ############################################################# #
        # ---------------- ROS Data Recording Service ----------------- #
        # ############################################################# #
        self.recording_service = None

        self.enable_recording_service = bool(
            self.config.get("enable_recording_service", True)
        )
        self.enable_ros_interfaces = bool(
            self.config.get("enable_ros_interfaces", True)
        )

        self.recording_service_name = self.config.get(
            "recording_service_name",
            "/mujoco_hdf5_recording/set_recording"
        )

        # ############################################################ #
        # ---------------- Target smoothing / safety ----------------- #
        # ############################################################ #

        self.max_joint_velocity = float(self.config.get("max_joint_velocity", 0.5))  # rad/s
        self.max_joint_step_qpos = float(self.config.get("max_joint_step_qpos", 0.015))  # rad/frame for qpos debug mode

        self.enable_force_velocity_scaling = bool(
            self.config.get("enable_force_velocity_scaling", False)
        )
        self.force_velocity_medium_threshold = float(
            self.config.get("force_velocity_medium_threshold", 40.0)
        )
        self.force_velocity_high_threshold = float(
            self.config.get("force_velocity_high_threshold", 80.0)
        )
        self.force_velocity_medium_scale = float(
            self.config.get("force_velocity_medium_scale", 0.4)
        )
        self.force_velocity_high_scale = float(
            self.config.get("force_velocity_high_scale", 0.15)
        )
        self.enable_high_force_hold = bool(
            self.config.get("enable_high_force_hold", False)
        )
        self.high_force_hold_threshold = float(
            self.config.get("high_force_hold_threshold", 100.0)
        )
        self.high_force_hold_dwell_time = float(
            self.config.get("high_force_hold_dwell_time", 0.15)
        )
        self.high_force_hold_release_ratio = float(
            self.config.get("high_force_hold_release_ratio", 0.8)
        )

        if not (
            0.0 <= self.force_velocity_medium_threshold
            < self.force_velocity_high_threshold
        ):
            raise ValueError(
                "force velocity thresholds must satisfy "
                "0 <= medium_threshold < high_threshold"
            )
        if not (
            0.0 < self.force_velocity_high_scale
            <= self.force_velocity_medium_scale
            <= 1.0
        ):
            raise ValueError(
                "force velocity scales must satisfy "
                "0 < high_scale <= medium_scale <= 1"
            )
        if self.high_force_hold_threshold <= 0.0:
            raise ValueError("high_force_hold_threshold must be positive")
        if self.high_force_hold_dwell_time < 0.0:
            raise ValueError("high_force_hold_dwell_time must be non-negative")
        if not 0.0 < self.high_force_hold_release_ratio < 1.0:
            raise ValueError(
                "high_force_hold_release_ratio must be between 0 and 1"
            )

        self.force_velocity_scale = 1.0
        self.force_filter_last_force_norm = 0.0
        self.high_force_hold_start_time = None
        self.high_force_hold_active = False




        # ############################################################ #
        # # ---------------- Joint torque alarm -------------------- # #
        # ############################################################ #
        self.enable_joint_torque_alarm = bool(
            self.config.get("enable_joint_torque_alarm", True)
        )

        self.joint_torque_alarm_only_when_recording = bool(
            self.config.get("joint_torque_alarm_only_when_recording", True)
        )

        self.joint_torque_alarm_latched = bool(
            self.config.get("joint_torque_alarm_latched", True)
        )

        raw_torque_limits = self.config.get("joint_torque_limits", 20.0)
        n_arm_joints = len(self.arm_joint_names)

        if isinstance(raw_torque_limits, (int, float)):
            self.joint_torque_limits = np.full(
                n_arm_joints,
                float(raw_torque_limits),
                dtype=float,
            )
        else:
            self.joint_torque_limits = np.asarray(
                raw_torque_limits,
                dtype=float,
            )

        if self.joint_torque_limits.shape[0] != n_arm_joints:
            raise ValueError(
                "joint_torque_limits must be a scalar or a list with "
                f"{n_arm_joints} values."
            )

        self.joint_torque_alarm_peg_rgba = np.asarray(
            self.config.get("joint_torque_alarm_peg_rgba", [0.0, 1.0, 0.0, 1.0]),
            dtype=float,
        )

        self.reset_joint_torque_alarm_on_record_start = bool(
            self.config.get("reset_joint_torque_alarm_on_record_start", True)
        )

        self.joint_torque_alarm_active = False
        self.joint_torque_alarm_first_time = None
        self.joint_torque_alarm_first_joint = ""
        self.joint_torque_alarm_first_value = 0.0
        self.joint_torque_alarm_first_limit = 0.0
        self.joint_torque_alarm_last_values = np.zeros(n_arm_joints, dtype=float)




        # ############################################################ #
        # # ---------------- FT wrench alarm ----------------------- # #
        # ############################################################ #
        self.enable_ft_wrench_alarm = bool(
            self.config.get("enable_ft_wrench_alarm", True)
        )
        self.ft_wrench_alarm_only_when_recording = bool(
            self.config.get("ft_wrench_alarm_only_when_recording", True)
        )
        self.ft_wrench_alarm_latched = bool(
            self.config.get("ft_wrench_alarm_latched", True)
        )
        self.reset_ft_wrench_alarm_on_record_start = bool(
            self.config.get("reset_ft_wrench_alarm_on_record_start", True)
        )

        self.ft_force_norm_limit = float(
            self.config.get("ft_force_norm_limit", 40.0)
        )
        self.ft_torque_norm_limit = float(
            self.config.get("ft_torque_norm_limit", 3.0)
        )
        self.ft_wrench_alarm_dwell_time = float(
            self.config.get("ft_wrench_alarm_dwell_time", 0.15)
        )

        self.ft_wrench_alarm_peg_rgba = np.asarray(
            self.config.get("ft_wrench_alarm_peg_rgba", [0.0, 0.2, 1.0, 1.0]),
            dtype=float,
        )
        self.both_alarm_peg_rgba = np.asarray(
            self.config.get("both_alarm_peg_rgba", [0.8, 0.0, 1.0, 1.0]),
            dtype=float,
        )

        self.ft_wrench_alarm_freeze_on_trigger = bool(
            self.config.get("ft_wrench_alarm_freeze_on_trigger", False)
        )

        self.ft_wrench_alarm_active = False
        self.ft_wrench_alarm_start_time = None
        self.ft_wrench_alarm_first_time = None
        self.ft_wrench_alarm_first_force_norm = 0.0
        self.ft_wrench_alarm_first_torque_norm = 0.0
        self.ft_wrench_alarm_reason = ""





        # ############################################################ #
        # # ---------------- Task success auto-stop ---------------- # #
        # ############################################################ #
        self.enable_task_success_auto_stop = bool(
            self.config.get("enable_task_success_auto_stop", True)
        )

        self.task_success_only_when_recording = bool(
            self.config.get("task_success_only_when_recording", True)
        )

        self.task_success_pending_manual_review = bool(
            self.config.get("task_success_pending_manual_review", True)
        )
        self.task_success_auto_stop_recording = bool(
            self.config.get("task_success_auto_stop_recording", True)
        )

        self.task_success_stop_accepting_teleop = bool(
            self.config.get("task_success_stop_accepting_teleop", True)
        )

        self.task_success_peg_site_name = self.config.get(
            "task_success_peg_site_name",
            "peg_tip_site",
        )

        self.task_success_hole_site_name = self.config.get(
            "task_success_hole_site_name",
            "hole_goal_site",
        )

        self.task_success_distance = float(
            self.config.get("task_success_distance", 0.006)
        )

        self.task_success_dwell_time = float(
            self.config.get("task_success_dwell_time", 0.15)
        )

        self.task_success_terminal_hold_time = float(
            self.config.get("task_success_terminal_hold_time", 1.0)
        )

        self.task_success_blend_to_qpos_time = float(
            self.config.get("task_success_blend_to_qpos_time", 0.2)
        )

        self.task_success_peg_site_id = mujoco.mj_name2id(
            self.model,
            mujoco.mjtObj.mjOBJ_SITE,
            self.task_success_peg_site_name,
        )

        self.task_success_hole_site_id = mujoco.mj_name2id(
            self.model,
            mujoco.mjtObj.mjOBJ_SITE,
            self.task_success_hole_site_name,
        )

        if self.enable_task_success_auto_stop:
            print("[TaskSuccessAutoStop] enable:", self.enable_task_success_auto_stop)
            print(
                "[TaskSuccessAutoStop] peg site:",
                self.task_success_peg_site_name,
                self.task_success_peg_site_id,
            )
            print(
                "[TaskSuccessAutoStop] hole site:",
                self.task_success_hole_site_name,
                self.task_success_hole_site_id,
            )
            print("[TaskSuccessAutoStop] distance:", self.task_success_distance)
            print("[TaskSuccessAutoStop] dwell time:", self.task_success_dwell_time)
            print("[TaskSuccessAutoStop] terminal hold:", self.task_success_terminal_hold_time)
            print("[TaskSuccessAutoStop] blend to qpos:", self.task_success_blend_to_qpos_time)

            if self.task_success_peg_site_id == -1:
                raise ValueError(
                    f"Task success peg site not found: {self.task_success_peg_site_name}"
                )

            if self.task_success_hole_site_id == -1:
                raise ValueError(
                    f"Task success hole site not found: {self.task_success_hole_site_name}"
                )

        self.task_success_accumulated_time = 0.0
        self.task_success_last_check_time = None
        self.task_success_triggered = False

        self.terminal_hold_active = False
        self.terminal_hold_start_time = None
        self.terminal_hold_command_start = None
        self.terminal_hold_command_goal = None
        self.terminal_hold_stop_started = False

        self.pending_auto_completed_review = False
        self.pending_auto_completed_episode_path = ""
        self.pending_auto_completed_episode_dir = ""
        





        # ######################################################### #
        # ---------------- Hole position sampling ---------------- #
        # ######################################################### #

        legacy_randomization_enabled = bool(
            self.config.get("enable_hole_randomization", False)
        )
        default_sampling_mode = (
            "uniform_random" if legacy_randomization_enabled else "fixed"
        )
        self.hole_sampling_mode = str(
            self.config.get("hole_sampling_mode", default_sampling_mode)
        ).strip().lower()
        if self.hole_sampling_mode not in {"grid", "uniform_random", "fixed"}:
            raise ValueError(
                "hole_sampling_mode must be grid, uniform_random, or fixed; "
                f"got {self.hole_sampling_mode!r}"
            )

        self.hole_body_name = self.config.get(
            "hole_body_name",
            self.config.get("hole_random_body_name", "wall_task"),
        )
        self.hole_body_id = mujoco.mj_name2id(
            self.model,
            mujoco.mjtObj.mjOBJ_BODY,
            self.hole_body_name,
        )
        if self.hole_sampling_mode != "fixed" and self.hole_body_id == -1:
            raise ValueError(
                f"Hole sampling body not found in MuJoCo model: {self.hole_body_name}"
            )

        if self.hole_body_id != -1:
            self.hole_nominal_body_pos = self.model.body_pos[
                self.hole_body_id
            ].copy()
        else:
            self.hole_nominal_body_pos = np.zeros(3, dtype=float)

        self.hole_grid_advance_policy = str(
            self.config.get("hole_grid_advance_policy", "on_keep")
        ).strip().lower()
        if self.hole_grid_advance_policy not in {"on_keep", "on_attempt"}:
            raise ValueError(
                "hole_grid_advance_policy must be on_keep or on_attempt; "
                f"got {self.hole_grid_advance_policy!r}"
            )

        self.hole_grid_scheduler = None
        if self.hole_sampling_mode == "grid":
            self.hole_grid_scheduler = HoleGridScheduler(
                rows=int(self.config.get("hole_grid_rows", 5)),
                cols=int(self.config.get("hole_grid_cols", 5)),
                x_range=self.config.get("hole_grid_x_range", [-0.06, 0.06]),
                y_offset=float(self.config.get("hole_grid_y_offset", 0.0)),
                z_range=self.config.get("hole_grid_z_range", [-0.06, 0.06]),
                sample_mode=self.config.get("hole_grid_sample_mode", "center"),
                traversal_order=self.config.get(
                    "hole_grid_traversal_order",
                    "shuffled",
                ),
                seed=self.config.get("hole_grid_seed", 42),
                start_cycle=int(self.config.get("hole_grid_start_cycle", 0)),
                start_index=int(self.config.get("hole_grid_start_index", 0)),
            )

        self.hole_random_x_range = self.config.get(
            "hole_random_x_range", [-0.01, 0.01]
        )
        self.hole_random_y_range = self.config.get(
            "hole_random_y_range", [0.0, 0.0]
        )
        self.hole_random_z_range = self.config.get(
            "hole_random_z_range", [-0.01, 0.01]
        )
        self.hole_random_seed = self.config.get("hole_random_seed", None)
        self.hole_rng = np.random.default_rng(self.hole_random_seed)

        self.current_hole_sample = {
            "enabled": False,
            "sampling_mode": self.hole_sampling_mode,
            "body_name": self.hole_body_name,
            "nominal_body_pos": self.hole_nominal_body_pos.tolist(),
            "offset_xyz": [0.0, 0.0, 0.0],
            "body_pos": self.hole_nominal_body_pos.tolist(),
        }
        # Kept as a compatibility alias for downstream/debug code.
        self.last_hole_randomization = dict(self.current_hole_sample)





        # #################################### #
        # ----- Launching Initialization ----- #
        # #################################### #

        self.launch_viewer = bool(self.config.get("launch_viewer", True))
        self.viewer_start_wait = float(self.config.get("viewer_start_wait", 1.0))

        self.running = False
        self.viewer_running = False
        self.runtime_activated = False
        self.lock = threading.RLock()
        self.vis_thread: Optional[threading.Thread] = None
        self.viewer_handle = None
        self.stop_event = threading.Event()

        # Offscreen rendering reconstructs small immutable state snapshots in
        # a worker-owned MjData. The physics thread never waits for OpenGL or
        # OpenCV and remains the sole owner of the live self.data object.
        self.render_queue_size = max(
            1, int(self.config.get("render_queue_size", 4))
        )
        self.render_queue: queue.Queue = queue.Queue(
            maxsize=self.render_queue_size
        )
        self.render_thread: Optional[threading.Thread] = None
        self.render_stop_event = threading.Event()
        self.render_ready_event = threading.Event()
        self.render_start_error: Optional[str] = None
        self.render_start_timeout = max(
            0.1, float(self.config.get("render_start_timeout", 10.0))
        )
        self.next_camera_display_wall_time = 0.0
        self.render_snapshot_drops = 0
        self.render_error_count = 0
        self.physics_step_count = 0
        self.physics_overrun_count = 0
        self.physics_max_lag = 0.0

        # State targets.
        self.target_joints = [0.0] * len(self.joint_names)
        self.command_joints = [0.0] * len(self.joint_names)

        # Initial pose.
        # initial_arm_joints = self.config.get(
        #     "initial_arm_joints",
        #     [-0.046, -0.2, 0.0, 1.6, -1.32, 0.005, 0.005],
        # )
        # initial_internal = self._convert_arm_command_to_internal(initial_arm_joints)
        # self._set_internal_joint_targets(initial_internal, immediate=True)
        # self._hard_set_qpos(self.command_joints)
        # self._apply_actuator_targets(self.command_joints)
        # mujoco.mj_forward(self.model, self.data)
        # Initial pose.
        self.initial_arm_joints_external = list(
            self.config.get(
                "initial_arm_joints",
                [-0.046, -0.2, 0.0, 1.6, -1.32, 0.005, 0.005],
            )
        )

        self.initial_arm_joints_internal = self._convert_arm_command_to_internal(
            self.initial_arm_joints_external
        )

        self.reset_arm_on_record_start = bool(
            self.config.get("reset_arm_on_record_start", True)
        )

        self.reset_ignore_teleop_duration = float(
            self.config.get("reset_ignore_teleop_duration", 0.5)
        )

        self.ignore_teleop_until_wall_time = 0.0

        self._set_internal_joint_targets(
            self.initial_arm_joints_internal,
            immediate=True,
        )

        self._hard_set_qpos(self.command_joints)
        self._apply_actuator_targets(self.command_joints)
        mujoco.mj_forward(self.model, self.data)

        print("MuJoCo peg-tool 仿真器初始化完成")
        print(f"控制模式: {self.control_mode}")
        print(f"MuJoCo timestep: {self.sim_timestep:.6f} s")
        print(f"max_joint_velocity: {self.max_joint_velocity:.3f} rad/s")

        if not self.config.get("defer_runtime_activation", False):
            self.activate_runtime()

    def activate_runtime(self) -> None:
        """Prepare the task and expose runtime services exactly once.

        TeleopSystemMujoco defers this call until ArmTeleop has registered its
        stop/recalibrate/start services. Standalone controller users retain the
        old constructor behavior unless they explicitly request deferral.
        """
        if self.runtime_activated:
            return

        # Prepare the first task position before an episode or service request.
        self.prepare_hole_for_episode()

        if self.hdf5_auto_start and self.hdf5_recorder is not None:
            with self.lock:
                self.hdf5_recorder.start_episode(
                    label=self.config.get("hdf5_episode_label", "teleop"),
                    episode_metadata=self.current_hole_sample,
                )

        if self.config.get("auto_start", True):
            print("启动 MuJoCo 仿真线程...")
            self.start_simulation()

        # Publish the recording service last, when the task, teleoperation
        # services, and simulation are ready to accept an episode request.
        if (
            self.enable_ros_interfaces
            and self.enable_recording_service
            and self.hdf5_recorder is not None
        ):
            import rospy
            from arm_teleop.srv import SetRecording

            self.recording_service = rospy.Service(
                self.recording_service_name,
                SetRecording,
                self._handle_recording_service,
            )
            print(f"[Recording Service] Ready: {self.recording_service_name}")

        self.runtime_activated = True
    

    def _call_teleop_trigger_service(self, service_name: str, description: str) -> bool:
        """
        Call an arm_teleop Trigger service.

        Used to stop/recalibrate/start arm teleoperation around each recording episode.
        """
        if not getattr(self, "teleop_controlled_by_recording", True):
            return True
        if not self.enable_ros_interfaces:
            return True

        try:
            import rospy
            from std_srvs.srv import Trigger

            rospy.wait_for_service(service_name, timeout=3.0)
            proxy = rospy.ServiceProxy(service_name, Trigger)
            resp = proxy()

            if not resp.success:
                print(
                    f"[Episode Teleop] {description} failed: {resp.message}"
                )
                return False

            print(
                f"[Episode Teleop] {description} success: {resp.message}"
            )
            return True

        except Exception as e:
            print(
                f"[Episode Teleop] {description} service error "
                f"({service_name}): {e}"
            )
            return False
    
    def _auto_stop_recording_for_task_success(self):
        """
        Stop HDF5 recording after terminal hold.

        The episode is temporarily kept, then keyboard client can request
        keep/discard through the normal stop service.
        """
        try:
            if self.hdf5_recorder is None:
                print("[TaskSuccessAutoStop] HDF5 recorder is None.")
                return

            if not getattr(self.hdf5_recorder, "active", False):
                print("[TaskSuccessAutoStop] No active HDF5 recording.")
                return

            with self.lock:
                try:
                    self.hdf5_recorder.add_event("auto_stop_task_success")
                except Exception as e:
                    print(f"[TaskSuccessAutoStop] Failed to add auto-stop event: {e}")

                hdf5_path = self.hdf5_recorder.stop_episode(
                    status="auto_stop_task_success"
                )

            episode_path = str(hdf5_path) if hdf5_path is not None else ""
            episode_dir = str(hdf5_path.parent) if hdf5_path is not None else ""

            self.pending_auto_completed_review = True
            self.pending_auto_completed_episode_path = episode_path
            self.pending_auto_completed_episode_dir = episode_dir

            print(
                "[TaskSuccessAutoStop] Recording auto-stopped and temporarily kept.\n"
                f"  episode_path = {episode_path}\n"
                "  Press Enter in the keyboard client, then choose keep/discard."
            )

        except Exception as e:
            print(f"[TaskSuccessAutoStop] Auto-stop failed: {e}")

    def _reset_after_episode_stop(self):
        """
        Reset controller state and arm pose after an episode has stopped.

        Important:
        Call this only after HDF5 recording is inactive,
        so reset motion will not be recorded into the dataset.
        """
        self.accept_teleop_commands = False

        # Clear terminal-hold / task-success state first.
        with self.lock:
            self._reset_task_success_state_locked()

        if getattr(self, "reset_arm_on_record_stop", True):
            self.reset_arm_to_initial_pose()

    
    def _handle_recording_service(self, req):
        """
        ROS service callback for starting/stopping HDF5 episode recording.

        req.record = True:
            start a new episode

        req.record = False:
            stop current episode
            if req.keep is False, delete the generated episode folder
        """
        from arm_teleop.srv import SetRecordingResponse

        if self.hdf5_recorder is None:
            return SetRecordingResponse(
                success=False,
                active=False,
                message="HDF5 recorder is not initialized.",
                episode_path="",
            )

        try:
            # Start recording
            if req.record:
                if self.hdf5_recorder.active:
                    current_path = ""
                    if self.hdf5_recorder.hdf5_path is not None:
                        current_path = str(self.hdf5_recorder.hdf5_path)

                    return SetRecordingResponse(
                        success=False,
                        active=True,
                        message="Recording is already active.",
                        episode_path=current_path,
                    )

                label = req.label.strip() if req.label.strip() else "teleop"

                # 0. episode 准备阶段，拒绝任何旧的遥操作命令
                self.accept_teleop_commands = False

                # 1. 停止旧遥操作线程
                ok = self._call_teleop_trigger_service(
                    self.teleop_stop_service_name,
                    "stop teleoperation before record start",
                )
                if not ok:
                    return SetRecordingResponse(
                        success=False,
                        active=False,
                        message="Failed to stop old teleoperation before recording.",
                        episode_path="",
                    )

                # 2. reset MuJoCo 机械臂到默认初始位置
                if getattr(self, "reset_arm_on_record_start", True):
                    self.reset_arm_to_initial_pose()
                
                # 2.5 clear previous torque alarm before this episode
                if self.reset_joint_torque_alarm_on_record_start:
                    self.reset_joint_torque_alarm()
                if getattr(self, "reset_ft_wrench_alarm_on_record_start", True):
                    self.reset_ft_wrench_alarm()
                
                # 2.6 reset task success / terminal hold state
                with self.lock:
                    self._reset_task_success_state_locked()

                # 3. 当前孔位已在系统启动或上一条 episode 审核后准备好。

                # 4. 重新标定当前 Vision Pro 手部参考
                ok = self._call_teleop_trigger_service(
                    self.teleop_recalibrate_service_name,
                    "recalibrate teleoperation reference",
                )
                if not ok:
                    return SetRecordingResponse(
                        success=False,
                        active=False,
                        message="Failed to recalibrate teleoperation.",
                        episode_path="",
                    )

                # 5. 开始 HDF5 记录
                with self.lock:
                    hdf5_path = self.hdf5_recorder.start_episode(
                        label=label,
                        episode_metadata=self.current_hole_sample,
                    )

                # 6. 开启 controller 命令入口
                self.accept_teleop_commands = True

                # 7. 启动新一轮遥操作线程
                ok = self._call_teleop_trigger_service(
                    self.teleop_start_service_name,
                    "start teleoperation for new episode",
                )

                if not ok:
                    self.accept_teleop_commands = False

                    if self.hdf5_recorder.active:
                        with self.lock:
                            self.hdf5_recorder.stop_episode(
                                status="teleop_start_failed"
                            )

                    return SetRecordingResponse(
                        success=False,
                        active=False,
                        message="Failed to start teleoperation after recording started.",
                        episode_path=str(hdf5_path) if hdf5_path is not None else "",
                    )

                return SetRecordingResponse(
                    success=True,
                    active=True,
                    message=f"Started HDF5 recording and teleoperation: {label}",
                    episode_path=str(hdf5_path) if hdf5_path is not None else "",
                )
            # Stop recording
            else:
                # If task-success auto-stop has already stopped and temporarily kept
                # the episode, this stop request is treated as manual review.
                if (
                    not self.hdf5_recorder.active
                    and getattr(self, "pending_auto_completed_review", False)
                ):
                    episode_path = self.pending_auto_completed_episode_path
                    episode_dir = self.pending_auto_completed_episode_dir

                    if req.keep:
                        # HDF5 已经 inactive，reset 不会污染数据。
                        self._reset_after_episode_stop()
                        next_hole = self.finalize_hole_after_episode(keep=True)

                        return SetRecordingResponse(
                            success=True,
                            active=False,
                            message=(
                                "Auto-completed episode kept after manual review. "
                                f"Next hole: {next_hole.get('hole_grid_cell_label', 'n/a')}."
                            ),
                            episode_path=episode_path,
                        )

                    # discard pending auto-completed episode
                    try:
                        # 先 reset，让机械臂立刻回初始位姿。
                        self._reset_after_episode_stop()
                        retry_hole = self.finalize_hole_after_episode(keep=False)

                        # 再删除 episode 文件夹，避免删除大文件导致 reset 延迟。
                        if episode_dir:
                            shutil.rmtree(episode_dir, ignore_errors=True)

                        return SetRecordingResponse(
                            success=True,
                            active=False,
                            message=(
                                "Auto-completed episode discarded after manual review. "
                                f"Retry hole: {retry_hole.get('hole_grid_cell_label', 'n/a')}."
                            ),
                            episode_path=episode_path,
                        )

                    except Exception as e:
                        return SetRecordingResponse(
                            success=False,
                            active=False,
                            message=f"Failed to discard auto-completed episode: {e}",
                            episode_path=episode_path,
                        )
                # if (
                #     not self.hdf5_recorder.active
                #     and getattr(self, "pending_auto_completed_review", False)
                # ):
                #     episode_path = self.pending_auto_completed_episode_path
                #     episode_dir = self.pending_auto_completed_episode_dir

                #     if req.keep:
                #         self.pending_auto_completed_review = False
                #         self.pending_auto_completed_episode_path = ""
                #         self.pending_auto_completed_episode_dir = ""

                #         # After review, reset arm if configured.
                #         if getattr(self, "reset_arm_on_record_stop", True):
                #             self.reset_arm_to_initial_pose()

                #         return SetRecordingResponse(
                #             success=True,
                #             active=False,
                #             message=(
                #                 "Auto-completed episode kept after manual review."
                #             ),
                #             episode_path=episode_path,
                #         )

                #     # discard pending auto-completed episode
                #     try:
                #         if episode_dir:
                #             shutil.rmtree(episode_dir, ignore_errors=True)

                #         self.pending_auto_completed_review = False
                #         self.pending_auto_completed_episode_path = ""
                #         self.pending_auto_completed_episode_dir = ""

                #         if getattr(self, "reset_arm_on_record_stop", True):
                #             self.reset_arm_to_initial_pose()

                #         return SetRecordingResponse(
                #             success=True,
                #             active=False,
                #             message=(
                #                 "Auto-completed episode discarded after manual review."
                #             ),
                #             episode_path=episode_path,
                #         )

                #     except Exception as e:
                #         return SetRecordingResponse(
                #             success=False,
                #             active=False,
                #             message=f"Failed to discard auto-completed episode: {e}",
                #             episode_path=episode_path,
                #         )

                if not self.hdf5_recorder.active:
                    last_path = ""
                    if self.hdf5_recorder.hdf5_path is not None:
                        last_path = str(self.hdf5_recorder.hdf5_path)
                    return SetRecordingResponse(
                        success=False,
                        active=False,
                        message="No active recording episode.",
                        episode_path=last_path,

                    )
                
                # Normal active-recording stop path.
                # 到这里说明：
                #   1. 不是 pending auto-completed review
                #   2. hdf5_recorder.active == True
                # 所以这是普通手动停止 recording。

                # 0. 立刻拒绝新的遥操作命令
                self.accept_teleop_commands = False

                # 1. 停止遥操作线程
                self._call_teleop_trigger_service(
                    self.teleop_stop_service_name,
                    "stop teleoperation before record stop",
                )

                # 2. 生成力矩报警信息
                torque_alarm_msg = ""
                if getattr(self, "joint_torque_alarm_active", False):
                    torque_alarm_msg = (
                        " Joint torque alarm was triggered: "
                        f"{self.joint_torque_alarm_first_joint}, "
                        f"{self.joint_torque_alarm_first_value:.3f} Nm > "
                        f"{self.joint_torque_alarm_first_limit:.3f} Nm."
                    )
                quality_alarm_msg = self._make_quality_alarm_message()

                # 3. 先保存 stop 前的路径，避免 stop_episode 返回 None 时丢失路径
                hdf5_path_before_stop = self.hdf5_recorder.hdf5_path

                with self.lock:
                    hdf5_path = self.hdf5_recorder.stop_episode(
                        status="manual_keep" if req.keep else "manual_discard"
                    )

                if hdf5_path is None:
                    hdf5_path = hdf5_path_before_stop

                episode_path = str(hdf5_path) if hdf5_path is not None else ""

                # 4. reset 机械臂回默认位置。这个动作不进入 HDF5 数据。
                # if getattr(self, "reset_arm_on_record_stop", True):
                #     self.reset_arm_to_initial_pose()
                self._reset_after_episode_stop()

                next_hole = self.finalize_hole_after_episode(keep=bool(req.keep))

                # 5. 不保留则删除 episode 文件夹
                if not req.keep:
                    try:
                        if hdf5_path is not None:
                            episode_dir = hdf5_path.parent
                            if episode_dir.exists():
                                shutil.rmtree(episode_dir, ignore_errors=True)

                        return SetRecordingResponse(
                            success=True,
                            active=False,
                            message=(
                                "Stopped recording, stopped teleoperation, reset arm, "
                                "and discarded this episode. "
                                f"Retry hole: {next_hole.get('hole_grid_cell_label', 'n/a')}."
                                + quality_alarm_msg
                            ),
                            episode_path=episode_path,
                        )

                    except Exception as e:
                        return SetRecordingResponse(
                            success=False,
                            active=False,
                            message=(
                                f"Stopped recording, but failed to discard episode: {e}"
                            ),
                            episode_path=episode_path,
                        )

                # 6. 保留 episode
                return SetRecordingResponse(
                    success=True,
                    active=False,
                    message=(
                        "Stopped recording, stopped teleoperation, reset arm, "
                        "and kept this episode. "
                        f"Next hole: {next_hole.get('hole_grid_cell_label', 'n/a')}."
                        + quality_alarm_msg
                    ),
                    episode_path=episode_path,
                )
            # Stop recording
            # else:
            #     if not self.hdf5_recorder.active:
            #         last_path = ""
            #         if self.hdf5_recorder.hdf5_path is not None:
            #             last_path = str(self.hdf5_recorder.hdf5_path)

            #         return SetRecordingResponse(
            #             success=False,
            #             active=False,
            #             message="No active recording episode.",
            #             episode_path=last_path,
            #         )

            #     # 0. 立刻拒绝新的遥操作命令
            #     self.accept_teleop_commands = False

            #     # 1. 停止遥操作线程
            #     self._call_teleop_trigger_service(
            #         self.teleop_stop_service_name,
            #         "stop teleoperation before record stop",
            #     )

            #     # 2. 停止 HDF5 记录

            #     torque_alarm_msg = ""
            #     if getattr(self, "joint_torque_alarm_active", False):
            #         torque_alarm_msg = (
            #             " Joint torque alarm was triggered: "
            #             f"{self.joint_torque_alarm_first_joint}, "
            #             f"{self.joint_torque_alarm_first_value:.3f} Nm > "
            #             f"{self.joint_torque_alarm_first_limit:.3f} Nm."
            #         )

            #     hdf5_path = self.hdf5_recorder.stop_episode(
            #         status="manual_keep" if req.keep else "manual_discard"
            #     )

            #     episode_path = str(hdf5_path) if hdf5_path is not None else ""

            #     # 3. reset 机械臂回默认位置。这个动作不进入 HDF5 数据。
            #     if getattr(self, "reset_arm_on_record_stop", True):
            #         self.reset_arm_to_initial_pose()

            #     # 4. 不保留则删除 episode 文件夹
            #     if not req.keep:
            #         if hdf5_path is not None:
            #             episode_dir = hdf5_path.parent
            #             if episode_dir.exists():
            #                 shutil.rmtree(episode_dir)

            #         return SetRecordingResponse(
            #             success=True,
            #             active=False,
            #             # message="Stopped recording, stopped teleoperation, reset arm, and discarded this episode.",
            #             message=(
            #                 "Stopped recording, stopped teleoperation, reset arm, "
            #                 "and discarded this episode."
            #                 + torque_alarm_msg
            #             ),
            #             episode_path=episode_path,
            #         )

            #     return SetRecordingResponse(
            #         success=True,
            #         active=False,
            #         # message="Stopped recording, stopped teleoperation, reset arm, and kept this episode.",
            #         message=(
            #             "Stopped recording, stopped teleoperation, reset arm, "
            #             "and kept this episode."
            #             + torque_alarm_msg
            #         ),
            #         episode_path=episode_path,
            #     )
            # if req.record:
            #     if self.hdf5_recorder.active:
            #         current_path = ""
            #         if self.hdf5_recorder.hdf5_path is not None:
            #             current_path = str(self.hdf5_recorder.hdf5_path)

            #         return SetRecordingResponse(
            #             success=False,
            #             active=True,
            #             message="Recording is already active.",
            #             episode_path=current_path,
            #         )

            #     label = req.label.strip() if req.label.strip() else "teleop"

            #     # Randomize hole position before starting recording
            #     if self.randomize_hole_on_record_start:
            #         self.randomize_hole_position()

            #     hdf5_path = self.hdf5_recorder.start_episode(label=label)

            #     return SetRecordingResponse(
            #         success=True,
            #         active=True,
            #         message=f"Started HDF5 recording: {label}",
            #         episode_path=str(hdf5_path) if hdf5_path is not None else "",
            #     )

            # # Stop recording
            # else:
            #     if not self.hdf5_recorder.active:
            #         last_path = ""
            #         if self.hdf5_recorder.hdf5_path is not None:
            #             last_path = str(self.hdf5_recorder.hdf5_path)

            #         return SetRecordingResponse(
            #             success=False,
            #             active=False,
            #             message="No active recording episode.",
            #             episode_path=last_path,
            #         )

            #     hdf5_path = self.hdf5_recorder.stop_episode(
            #         status="manual_keep" if req.keep else "manual_discard"
            #     )

            #     episode_path = str(hdf5_path) if hdf5_path is not None else ""

            #     if not req.keep:
            #         # Delete the entire episode folder.
            #         # hdf5_path = .../<episode_folder>/episode.hdf5
            #         if hdf5_path is not None:
            #             episode_dir = hdf5_path.parent
            #             if episode_dir.exists():
            #                 shutil.rmtree(episode_dir)

            #         return SetRecordingResponse(
            #             success=True,
            #             active=False,
            #             message="Stopped recording and discarded this episode.",
            #             episode_path=episode_path,
            #         )

            #     return SetRecordingResponse(
            #         success=True,
            #         active=False,
            #         message="Stopped recording and kept this episode.",
            #         episode_path=episode_path,
            #     )

        except Exception as e:
            return SetRecordingResponse(
                success=False,
                active=bool(getattr(self.hdf5_recorder, "active", False)),
                message=f"Recording service error: {e}",
                episode_path="",
            )
    
    
    
    # ------------------------------------------------------------------
    # Model information
    # ------------------------------------------------------------------
    def _get_joint_names(self) -> List[str]:
        joint_names = []
        for i in range(self.model.njnt):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, i)
            if name:
                joint_names.append(name)
        return joint_names

    def _create_actuator_map(self) -> Dict[str, Optional[int]]:
        actuator_map: Dict[str, Optional[int]] = {}
        for joint_name in self.joint_names:
            actuator_name = f"motor_{joint_name}"
            actuator_id = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_name
            )
            if actuator_id != -1:
                actuator_map[joint_name] = actuator_id
                print(f"关节 '{joint_name}' 映射到执行器 ID: {actuator_id}")
            else:
                actuator_map[joint_name] = None
                print(f"警告: 未找到关节 '{joint_name}' 的执行器")
        return actuator_map

    def get_current_joints(self) -> List[float]:
        joint_angles = []
        with self.lock:
            for joint_name in self.joint_names:
                joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
                if joint_id == -1:
                    joint_angles.append(0.0)
                    continue

                qpos_addr = self.model.jnt_qposadr[joint_id]
                if qpos_addr < len(self.data.qpos):
                    joint_angles.append(round(float(self.data.qpos[qpos_addr]), 3))
                else:
                    joint_angles.append(0.0)
        return joint_angles
    
    # --------------------------
    # Camera Rendering
    # --------------------------
    def set_cctv_frame_sink(
        self,
        sink: Optional[Callable[[np.ndarray], Any]],
    ) -> bool:
        """Attach an optional sink for the final CCTV BGR frame.

        The sink is called only from the render worker. Passing ``None``
        restores the original local-display/recording-only behavior.
        """
        if sink is None:
            self.cctv_frame_sink = None
            return True

        if self.cctv_camera not in self.monitor_camera_ids:
            camera_id = mujoco.mj_name2id(
                self.model,
                mujoco.mjtObj.mjOBJ_CAMERA,
                self.cctv_camera,
            )
            if camera_id == -1:
                print(
                    f"[Camera Monitor] VisionPro stream disabled: camera not "
                    f"found: {self.cctv_camera}"
                )
                return False
            self.monitor_camera_ids[self.cctv_camera] = camera_id

        self.cctv_frame_sink = sink
        return True

    def _live_camera_output_enabled(self) -> bool:
        return bool(self.show_camera_streams or self.cctv_frame_sink is not None)

    def _publish_cctv_frame(self, frame_bgr: np.ndarray) -> None:
        sink = self.cctv_frame_sink
        if sink is None:
            return
        try:
            sink(frame_bgr)
        except Exception as exc:
            # Video transport failures must never fail rendering, HDF5, or the
            # physics loop. Throttle logs while retaining an exact error count.
            self.cctv_frame_sink_error_count += 1
            now = time.monotonic()
            if (
                self.cctv_frame_sink_error_count == 1
                or now - self._last_cctv_frame_sink_error_log_time >= 5.0
            ):
                print(f"[VisionPro Video] Frame publish failed: {exc}")
                self._last_cctv_frame_sink_error_log_time = now

    def update_camera_stream_windows(
        self,
        frames_rgb: Dict[str, np.ndarray],
        render_data: mujoco.MjData,
    ):
        """
        Display an already-rendered camera snapshot.

        This is called only by the render worker. It performs no access to the
        live physics data and no OpenGL work.
        """
        if not self._live_camera_output_enabled():
            return

        frames = []
        feedback = self._get_force_feedback_snapshot(render_data)

        if self.separate_cctv_window or self.cctv_frame_sink is not None:
            cctv_bgr = self._render_cctv_window_bgr(
                camera_name=self.cctv_camera,
                feedback=feedback,
                frames_rgb=frames_rgb,
            )
            if cctv_bgr is not None:
                self._publish_cctv_frame(cctv_bgr)
                if self.show_camera_streams and self.separate_cctv_window:
                    self._ensure_cctv_window()
                    cv2.imshow(self.cctv_window_name, cctv_bgr)

        if not self.show_camera_streams:
            return

        for cam_name in self.monitor_camera_names:
            if (
                self.separate_cctv_window
                and not self.show_cctv_in_combined_panel
                and cam_name == self.cctv_camera
            ):
                continue

            bgr = self._render_display_camera_bgr(
                camera_name=cam_name,
                feedback=feedback,
                frames_rgb=frames_rgb,
            )
            if bgr is None:
                continue

            frames.append(bgr)

        if len(frames) > 0:
            # 如果只有一个相机，就单独显示
            if len(frames) == 1:
                panel = frames[0]
            else:
                # 横向拼接两个画面
                panel = np.hstack(frames)

            cv2.imshow("Task Camera Streams", panel)

        if (
            self.force_feedback_config.enabled
            and self.force_feedback_config.display_mode == "window"
            and feedback is not None
        ):
            cv2.imshow(
                self.force_feedback_config.window_name,
                make_force_feedback_hud(feedback, self.force_feedback_config),
            )
        cv2.waitKey(1)

    def _render_display_camera_bgr(
        self,
        camera_name: str,
        feedback=None,
        size=None,
        frames_rgb: Optional[Dict[str, np.ndarray]] = None,
    ):
        rgb = (frames_rgb or {}).get(camera_name)
        if rgb is None:
            return None

        # MuJoCo 返回 RGB，OpenCV 显示用 BGR。Overlay is display-only.
        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        if size is not None:
            bgr = cv2.resize(bgr, size, interpolation=cv2.INTER_LINEAR)

        if self._should_overlay_force_feedback(camera_name) and feedback is not None:
            draw_force_feedback_overlay(
                frame_bgr=bgr,
                feedback=feedback,
                config=self.force_feedback_config,
                camera_name=camera_name,
            )

        # 在左上角打上相机名称
        cv2.putText(
            bgr,
            camera_name,
            (15, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )

        return bgr

    def _render_cctv_window_bgr(
        self,
        camera_name: str,
        feedback=None,
        frames_rgb: Optional[Dict[str, np.ndarray]] = None,
    ):
        rgb = (frames_rgb or {}).get(camera_name)
        if rgb is None:
            return None

        # This is only the live display copy. HDF5 uses a separate renderer.
        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

        if self.cctv_window_preserve_aspect_ratio:
            bgr = resize_with_aspect_padding(
                frame_bgr=bgr,
                target_width=self.cctv_window_width,
                target_height=self.cctv_window_height,
                padding_color=self.cctv_window_padding_color,
            )
        else:
            bgr = cv2.resize(
                bgr,
                (self.cctv_window_width, self.cctv_window_height),
                interpolation=cv2.INTER_LINEAR,
            )

        if self._should_overlay_force_feedback(camera_name) and feedback is not None:
            draw_force_feedback_overlay(
                frame_bgr=bgr,
                feedback=feedback,
                config=self.force_feedback_config,
                camera_name=camera_name,
            )

        cv2.putText(
            bgr,
            camera_name,
            (15, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )

        return bgr

    def _ensure_cctv_window(self):
        if self.cctv_window_initialized:
            return

        try:
            cv2.namedWindow(self.cctv_window_name, cv2.WINDOW_NORMAL)
            if self.cctv_window_fullscreen:
                cv2.setWindowProperty(
                    self.cctv_window_name,
                    cv2.WND_PROP_FULLSCREEN,
                    cv2.WINDOW_FULLSCREEN,
                )
            else:
                cv2.resizeWindow(
                    self.cctv_window_name,
                    self.cctv_window_width,
                    self.cctv_window_height,
                )
        except Exception as e:
            print(f"[Camera Monitor] CCTV window setup failed: {e}")

        self.cctv_window_initialized = True

    def _render_pipeline_enabled(self) -> bool:
        return bool(
            self._live_camera_output_enabled()
            or (
                self.hdf5_recorder is not None
                and self.hdf5_recorder.record_images
            )
        )

    @staticmethod
    def _render_camera_set(
        renderer: mujoco.Renderer,
        render_data: mujoco.MjData,
        camera_names: List[str],
        camera_ids: Dict[str, int],
    ) -> Dict[str, np.ndarray]:
        frames: Dict[str, np.ndarray] = {}
        for camera_name in camera_names:
            camera_id = camera_ids.get(camera_name, -1)
            if camera_id == -1:
                continue
            renderer.update_scene(render_data, camera=camera_id)
            frames[camera_name] = np.asarray(renderer.render()).copy()
        return frames

    def _render_worker_loop(self) -> None:
        """Render copied simulation states and service display/recording sinks."""
        display_renderer = None
        recording_renderer = None
        shared_renderer = False

        try:
            render_model = copy.copy(self.model)
            render_data = mujoco.MjData(render_model)
            recorder = self.hdf5_recorder
            recording_enabled = bool(
                recorder is not None and recorder.record_images
            )
            same_resolution = bool(
                recording_enabled
                and recorder.image_width == self.camera_stream_width
                and recorder.image_height == self.camera_stream_height
            )

            if self._live_camera_output_enabled():
                display_renderer = mujoco.Renderer(
                    render_model,
                    height=self.camera_stream_height,
                    width=self.camera_stream_width,
                )

            if recording_enabled:
                if display_renderer is not None and same_resolution:
                    recording_renderer = display_renderer
                    shared_renderer = True
                else:
                    recording_renderer = mujoco.Renderer(
                        render_model,
                        height=recorder.image_height,
                        width=recorder.image_width,
                    )

            self.render_ready_event.set()

            while (
                not self.render_stop_event.is_set()
                or not self.render_queue.empty()
            ):
                try:
                    snapshot = self.render_queue.get(timeout=0.1)
                except queue.Empty:
                    continue

                recording_request_released = False
                try:
                    render_model.body_pos[:] = snapshot.body_pos
                    render_model.mat_rgba[:] = snapshot.mat_rgba
                    render_model.geom_rgba[:] = snapshot.geom_rgba
                    render_data.time = snapshot.time
                    render_data.qpos[:] = snapshot.qpos
                    render_data.qvel[:] = snapshot.qvel
                    render_data.act[:] = snapshot.act
                    render_data.ctrl[:] = snapshot.ctrl
                    render_data.qacc_warmstart[:] = snapshot.qacc_warmstart
                    render_data.qfrc_applied[:] = snapshot.qfrc_applied
                    render_data.xfrc_applied[:] = snapshot.xfrc_applied
                    render_data.mocap_pos[:] = snapshot.mocap_pos
                    render_data.mocap_quat[:] = snapshot.mocap_quat
                    render_data.userdata[:] = snapshot.userdata
                    mujoco.mj_forward(render_model, render_data)
                    # Preserve the exact post-step FT sample for the operator
                    # HUD; mj_forward reconstructs visuals but may solve
                    # contacts with a slightly different warm-start path.
                    render_data.sensordata[:] = snapshot.sensordata
                    display_frames: Dict[str, np.ndarray] = {}
                    recording_frames: Dict[str, np.ndarray] = {}

                    if shared_renderer:
                        names = []
                        if snapshot.display_requested:
                            names.extend(self.monitor_camera_ids.keys())
                        if snapshot.recording_request is not None:
                            names.extend(recorder.camera_ids.keys())
                        unique_names = list(dict.fromkeys(names))
                        camera_ids = dict(self.monitor_camera_ids)
                        camera_ids.update(recorder.camera_ids)
                        rendered = self._render_camera_set(
                            display_renderer,
                            render_data,
                            unique_names,
                            camera_ids,
                        )
                        if snapshot.display_requested:
                            display_frames = {
                                name: rendered[name]
                                for name in self.monitor_camera_ids
                                if name in rendered
                            }
                        if snapshot.recording_request is not None:
                            recording_frames = {
                                name: rendered[name]
                                for name in recorder.camera_ids
                                if name in rendered
                            }
                    else:
                        if snapshot.display_requested and display_renderer is not None:
                            display_frames = self._render_camera_set(
                                display_renderer,
                                render_data,
                                list(self.monitor_camera_ids.keys()),
                                self.monitor_camera_ids,
                            )
                        if (
                            snapshot.recording_request is not None
                            and recording_renderer is not None
                        ):
                            recording_frames = self._render_camera_set(
                                recording_renderer,
                                render_data,
                                list(recorder.camera_ids.keys()),
                                recorder.camera_ids,
                            )

                    if snapshot.recording_request is not None:
                        recorder.enqueue_image_sample(
                            snapshot.recording_request,
                            recording_frames,
                        )
                        recording_request_released = True

                    if snapshot.display_requested:
                        self.update_camera_stream_windows(
                            display_frames,
                            render_data,
                        )
                except Exception as exc:
                    self.render_error_count += 1
                    print(f"[RenderWorker] Snapshot failed: {exc}")
                    if (
                        snapshot.recording_request is not None
                        and not recording_request_released
                        and self.hdf5_recorder is not None
                    ):
                        self.hdf5_recorder.drop_image_request(
                            snapshot.recording_request,
                            reason=f"render_error: {exc}",
                        )
                finally:
                    self.render_queue.task_done()
        except Exception as exc:
            self.render_error_count += 1
            if not self.render_ready_event.is_set():
                self.render_start_error = str(exc)
                self.render_ready_event.set()
            print(f"[RenderWorker] Fatal error: {exc}")
        finally:
            while True:
                try:
                    snapshot = self.render_queue.get_nowait()
                except queue.Empty:
                    break
                if (
                    snapshot.recording_request is not None
                    and self.hdf5_recorder is not None
                ):
                    self.hdf5_recorder.drop_image_request(
                        snapshot.recording_request,
                        reason="render_worker_stopped",
                    )
                self.render_queue.task_done()
            if recording_renderer is not None and not shared_renderer:
                recording_renderer.close()
            if display_renderer is not None:
                display_renderer.close()
            cv2.destroyAllWindows()
            self.cctv_window_initialized = False

    def _start_render_worker(self) -> None:
        if not self._render_pipeline_enabled():
            return
        if self.render_thread is not None and self.render_thread.is_alive():
            return
        self.render_stop_event.clear()
        self.render_ready_event.clear()
        self.render_start_error = None
        self.render_thread = threading.Thread(
            target=self._render_worker_loop,
            name="MujocoRenderWorker",
            daemon=True,
        )
        self.render_thread.start()
        if not self.render_ready_event.wait(timeout=self.render_start_timeout):
            self.render_stop_event.set()
            self.render_thread.join(timeout=self.render_start_timeout)
            raise RuntimeError("Timed out starting MuJoCo render worker")
        if self.render_start_error is not None:
            self.render_stop_event.set()
            self.render_thread.join(timeout=self.render_start_timeout)
            raise RuntimeError(
                f"Unable to start MuJoCo render worker: {self.render_start_error}"
            )

    def _schedule_render_snapshot_locked(
        self,
        recording_request: Optional[ImageCaptureRequest],
    ) -> None:
        now = time.perf_counter()
        display_requested = bool(
            self._live_camera_output_enabled()
            and now >= self.next_camera_display_wall_time
        )
        if display_requested:
            self.next_camera_display_wall_time = now + self.camera_stream_period

        if not display_requested and recording_request is None:
            return

        if self.render_thread is None or not self.render_thread.is_alive():
            self.render_snapshot_drops += 1
            if recording_request is not None and self.hdf5_recorder is not None:
                self.hdf5_recorder.drop_image_request(
                    recording_request,
                    reason="render_worker_unavailable",
                )
            return

        snapshot = _RenderSnapshot(
            display_requested=display_requested,
            recording_request=recording_request,
            time=float(self.data.time),
            qpos=self.data.qpos.copy(),
            qvel=self.data.qvel.copy(),
            act=self.data.act.copy(),
            ctrl=self.data.ctrl.copy(),
            qacc_warmstart=self.data.qacc_warmstart.copy(),
            qfrc_applied=self.data.qfrc_applied.copy(),
            xfrc_applied=self.data.xfrc_applied.copy(),
            mocap_pos=self.data.mocap_pos.copy(),
            mocap_quat=self.data.mocap_quat.copy(),
            userdata=self.data.userdata.copy(),
            sensordata=self.data.sensordata.copy(),
            body_pos=self.model.body_pos.copy(),
            mat_rgba=self.model.mat_rgba.copy(),
            geom_rgba=self.model.geom_rgba.copy(),
        )
        try:
            self.render_queue.put_nowait(snapshot)
        except queue.Full:
            self.render_snapshot_drops += 1
            if recording_request is not None and self.hdf5_recorder is not None:
                self.hdf5_recorder.drop_image_request(
                    recording_request,
                    reason="render_queue_full",
                )

    def _should_overlay_force_feedback(self, camera_name: str) -> bool:
        cfg = self.force_feedback_config
        overlay_cameras = cfg.overlay_cameras
        if cfg.enable_task_force_guidance_hud:
            overlay_cameras = cfg.force_guidance_overlay_cameras

        return (
            cfg.enabled
            and cfg.display_mode == "overlay"
            and camera_name in overlay_cameras
        )

    def _get_force_feedback_snapshot(self, render_data: mujoco.MjData):
        cfg = self.force_feedback_config
        if not cfg.enabled or cfg.display_mode == "off":
            return None

        wrench, source_label = self._get_force_feedback_wrench(render_data)
        if wrench is None or len(wrench) < 6:
            return None

        force_sensor = np.asarray(wrench[:3], dtype=float)
        torque_sensor = np.asarray(wrench[3:6], dtype=float)
        if not np.all(np.isfinite(force_sensor)) or not np.all(np.isfinite(torque_sensor)):
            return None

        force_sensor = self.force_feedback_smoother.update(force_sensor)
        force_norm = float(np.linalg.norm(force_sensor))
        trend = self._update_force_feedback_trend(force_norm)
        R_ws = self._get_force_feedback_sensor_rotation_world(render_data)
        basis_right, basis_up, basis_label = self._get_force_guidance_basis_world(
            render_data
        )

        return compute_force_feedback(
            force_sensor=force_sensor,
            config=cfg,
            source_label=source_label,
            R_ws=R_ws,
            trend=trend,
            torque_sensor=torque_sensor,
            guidance_right_world=basis_right,
            guidance_up_world=basis_up,
            guidance_basis_label=basis_label,
        )

    def _get_force_feedback_wrench(self, render_data: mujoco.MjData):
        raw = raw_ft_wrench(self.model, render_data)
        if raw is None:
            return None, "raw"

        if not self.force_feedback_config.use_compensated_wrench:
            return raw, "raw"

        gravity = gravity_wrench_sensor_frame(
            model=self.model,
            data=render_data,
            ft_site_id=self.force_feedback_ft_sensor_site_id,
            tool_body_ids=self.force_feedback_gravity_tool_body_ids,
            gravity_world=self.force_feedback_gravity_world,
            sensor_sign=self.force_feedback_gravity_sensor_sign,
        )
        comp = compensated_ft_wrench(
            raw_wrench=raw,
            gravity_wrench=gravity,
            compensation_mode=self.force_feedback_compensation_mode,
        )

        if comp is not None and np.all(np.isfinite(comp)):
            return comp, "comp"

        if not self.force_feedback_raw_fallback_warned:
            print(
                "[ForceFeedbackHUD] Warning: compensated wrench unavailable; "
                "falling back to raw FT values for display."
            )
            self.force_feedback_raw_fallback_warned = True

        return raw, "raw"

    def _update_force_feedback_trend(self, force_norm: float) -> str:
        cfg = self.force_feedback_config
        now = time.perf_counter()
        self.force_feedback_history.append((now, float(force_norm)))

        window_sec = max(float(cfg.trend_window_sec), self.camera_stream_period)
        cutoff = now - window_sec
        self.force_feedback_history = [
            row for row in self.force_feedback_history if row[0] >= cutoff
        ]

        if len(self.force_feedback_history) < 2:
            return "STABLE"

        old_force = self.force_feedback_history[0][1]
        return trend_label(force_norm - old_force, cfg)

    def _get_force_feedback_sensor_rotation_world(
        self,
        render_data: mujoco.MjData,
    ):
        site_id = getattr(self, "force_feedback_ft_sensor_site_id", -1)
        if site_id == -1:
            return None

        try:
            _, R_ws = ft_sensor_pose_world(render_data, site_id)
            return R_ws
        except Exception:
            return None

    def _get_force_guidance_basis_world(self, render_data: mujoco.MjData):
        cfg = self.force_feedback_config

        if cfg.force_guidance_basis_mode == "sensor_debug":
            return None, None, "sensor_debug"

        if cfg.force_guidance_basis_mode == "camera_screen":
            basis = self._get_camera_screen_force_guidance_basis(render_data)
            if basis is not None:
                right, up = basis
                return right, up, "camera_screen"

            if not self.force_guidance_basis_fallback_warned:
                print(
                    "[ForceFeedbackHUD] Warning: camera_screen guidance basis "
                    "unavailable; falling back to manual_world."
                )
                self.force_guidance_basis_fallback_warned = True

            right, up = self._manual_force_guidance_basis()
            return right, up, "manual_world fallback"

        right, up = self._manual_force_guidance_basis()
        return right, up, cfg.force_guidance_basis_mode

    def _get_camera_screen_force_guidance_basis(
        self,
        render_data: mujoco.MjData,
    ):
        cfg = self.force_feedback_config
        cam_id = mujoco.mj_name2id(
            self.model,
            mujoco.mjtObj.mjOBJ_CAMERA,
            cfg.force_guidance_basis_camera,
        )
        if cam_id == -1:
            return None

        try:
            R_wc = render_data.cam_xmat[cam_id].copy().reshape(3, 3)
        except Exception:
            return None

        # MuJoCo camera xyaxes define local +X as image-right and +Y as image-up.
        right_world = R_wc[:, 0] * cfg.force_guidance_screen_right_sign
        up_world = R_wc[:, 1] * cfg.force_guidance_screen_up_sign

        return self._orthonormalize_guidance_basis(right_world, up_world)

    def _manual_force_guidance_basis(self):
        cfg = self.force_feedback_config
        return self._orthonormalize_guidance_basis(
            cfg.force_guidance_plane_right_world,
            cfg.force_guidance_plane_up_world,
        )

    def _orthonormalize_guidance_basis(self, right_world, up_world):
        axis = np.asarray(
            self.force_feedback_config.insertion_axis_world,
            dtype=float,
        ).reshape(3)
        axis_norm = np.linalg.norm(axis)
        if axis_norm < 1e-9:
            return None
        axis = axis / axis_norm

        right = self._project_vector_to_task_plane(right_world, axis)
        if right is None:
            return None

        up = self._project_vector_to_task_plane(up_world, axis)
        if up is None or abs(float(np.dot(right, up))) > 0.95:
            up = np.cross(axis, right)
            up_norm = np.linalg.norm(up)
            if up_norm < 1e-9:
                return None
            up = up / up_norm
        else:
            up = up - float(np.dot(up, right)) * right
            up_norm = np.linalg.norm(up)
            if up_norm < 1e-9:
                return None
            up = up / up_norm

        return right, up

    @staticmethod
    def _project_vector_to_task_plane(vec, axis):
        vec = np.asarray(vec, dtype=float).reshape(3)
        if not np.all(np.isfinite(vec)):
            return None
        projected = vec - float(np.dot(vec, axis)) * axis
        norm = np.linalg.norm(projected)
        if norm < 1e-9:
            return None
        return projected / norm

    

    # ------------------------------------------------------------------
    # Command conversion and target setting
    # ------------------------------------------------------------------
    def _convert_arm_command_to_internal(self, arm_target_joints: List[float]) -> List[float]:
        if len(arm_target_joints) != 7:
            raise ValueError(f"arm_target_joints 应为 7 个，当前为 {len(arm_target_joints)}")

        return [q * s for q, s in zip(arm_target_joints, self.arm_sign)]

    def _set_internal_joint_targets(self, internal_arm_joints: List[float], immediate: bool = False):
        if len(internal_arm_joints) != 7:
            raise ValueError("internal_arm_joints must have length 7")

        with self.lock:
            if len(self.target_joints) < 7:
                raise RuntimeError(f"当前模型关节数为 {len(self.target_joints)}，少于 7 个")

            self.target_joints[:7] = list(internal_arm_joints)

            if immediate:
                self.command_joints[:7] = list(internal_arm_joints)

    # def set_arm_positions(self, arm_target_joints: List[float]):
    #     """
    #     Public teleoperation interface.

    #     Input:
    #         arm_target_joints: 7 joint angles from IK / teleoperation.

    #     In actuator mode:
    #         This function only updates target_joints.
    #         The simulation thread rate-limits command_joints and sends them to actuators.

    #     In qpos mode:
    #         It also updates target_joints, and update_positions() will directly write qpos.
    #     """
    #     if len(arm_target_joints) != 7:
    #         print(f"错误: 机械臂目标关节数({len(arm_target_joints)})应为7个")
    #         return

    #     internal = self._convert_arm_command_to_internal(arm_target_joints)
    #     self._set_internal_joint_targets(internal, immediate=False)
    
    # def set_arm_positions(self, arm_target_joints: List[float]):
    #     """
    #     Public teleoperation interface.
    #     """
    #     if not getattr(self, "accept_teleop_commands", True):
    #         return

    #     if len(arm_target_joints) != 7:
    #         print(f"错误: 机械臂目标关节数({len(arm_target_joints)})应为7个")
    #         return

    #     internal = self._convert_arm_command_to_internal(arm_target_joints)
    #     self._set_internal_joint_targets(internal, immediate=False)

    def set_arm_positions(self, arm_target_joints: List[float]):
        """
        Public teleoperation interface.
        """
        if not getattr(self, "accept_teleop_commands", True):
            return

        # Ignore stale teleoperation commands immediately after reset.
        if time.time() < getattr(self, "ignore_teleop_until_wall_time", 0.0):
            return

        if len(arm_target_joints) != 7:
            print(f"错误: 机械臂目标关节数({len(arm_target_joints)})应为7个")
            return

        internal = self._convert_arm_command_to_internal(arm_target_joints)
        self._set_internal_joint_targets(internal, immediate=False)
    
    def reset_arm_to_initial_pose(self):
        """
        Reset the arm to the configured initial pose before starting a new episode.

        This reset is intended to happen before hdf5_recorder.start_episode(),
        so the recorded first frame is already the reset state.
        """
        if not hasattr(self, "initial_arm_joints_internal"):
            print("[Arm Reset] initial_arm_joints_internal not found, skip reset.")
            return

        with self.lock:
            self._set_internal_joint_targets(
                self.initial_arm_joints_internal,
                immediate=True,
            )

            self._hard_set_qpos(self.command_joints)
            self._apply_actuator_targets(self.command_joints)

            # Clear residual dynamics from the previous episode.
            if hasattr(self.data, "qacc"):
                self.data.qacc[:] = 0.0

            if hasattr(self.data, "qacc_warmstart"):
                self.data.qacc_warmstart[:] = 0.0

            if hasattr(self.data, "qfrc_applied"):
                self.data.qfrc_applied[:] = 0.0

            if hasattr(self.data, "xfrc_applied"):
                self.data.xfrc_applied[:] = 0.0

            mujoco.mj_forward(self.model, self.data)

            # Temporarily ignore stale teleoperation commands.
            self.ignore_teleop_until_wall_time = (
                time.time() + self.reset_ignore_teleop_duration
            )

        print(
            "[Arm Reset] Reset arm to initial pose: "
            f"{self.initial_arm_joints_external}"
        )


    def set_hand_positions(self, hand_target_joints: List[float]):
        """
        peg-tool model has no hand joints. This no-op keeps upper-level code compatible.
        """
        return

    # ------------------------------------------------------------------
    # Low-level state/control
    # ------------------------------------------------------------------
    def _hard_set_qpos(self, target_joints: List[float]):
        """
        Directly set qpos. Use only for reset/initialization or qpos debug mode.
        """
        if len(target_joints) != len(self.joint_names):
            print(
                f"错误: 目标关节数({len(target_joints)})与模型关节数"
                f"({len(self.joint_names)})不匹配"
            )
            return

        for i, joint_name in enumerate(self.joint_names):
            joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
            if joint_id == -1:
                continue

            qpos_addr = self.model.jnt_qposadr[joint_id]
            dof_addr = self.model.jnt_dofadr[joint_id]

            if qpos_addr < len(self.data.qpos):
                self.data.qpos[qpos_addr] = target_joints[i]
            if dof_addr < len(self.data.qvel):
                self.data.qvel[dof_addr] = 0.0

    def update_positions(self, target_joints: List[float]):
        """
        Kept for compatibility with the original controller.

        In contact/actuator mode you should not call this continuously from outside.
        The normal route is:
            set_arm_positions(...) -> target_joints -> actuator ctrl -> mj_step()

        If control_mode == qpos, this performs rate-limited direct qpos update for debugging.
        """
        if self.control_mode == "actuator":
            with self.lock:
                self.target_joints = list(target_joints)
            return

        with self.lock:
            limited = self._limit_qpos_step_locked(target_joints, self.max_joint_step_qpos)
            self.command_joints = limited
            self._hard_set_qpos(self.command_joints)
            mujoco.mj_forward(self.model, self.data)

    def _limit_qpos_step_locked(self, target_joints: List[float], max_step: float) -> List[float]:
        limited = []
        for current, target in zip(self.command_joints, target_joints):
            delta = target - current
            delta = max(-max_step, min(max_step, delta))
            limited.append(current + delta)
        return limited

    def _reset_force_velocity_filter_locked(self):
        self.force_velocity_scale = 1.0
        self.force_filter_last_force_norm = 0.0
        self.high_force_hold_start_time = None
        self.high_force_hold_active = False

    def _update_force_velocity_filter_locked(self):
        wrench = self._get_peg_ft_sensor_locked()
        if wrench is None or len(wrench) < 3:
            self.force_velocity_scale = 1.0
            self.high_force_hold_start_time = None
            self.high_force_hold_active = False
            return

        force = np.asarray(wrench[:3], dtype=float)
        if not np.all(np.isfinite(force)):
            self.force_velocity_scale = 1.0
            self.high_force_hold_start_time = None
            self.high_force_hold_active = False
            return

        force_norm = float(np.linalg.norm(force))
        self.force_filter_last_force_norm = force_norm

        if not self.enable_force_velocity_scaling:
            self.force_velocity_scale = 1.0
        elif force_norm >= self.force_velocity_high_threshold:
            self.force_velocity_scale = self.force_velocity_high_scale
        elif force_norm >= self.force_velocity_medium_threshold:
            self.force_velocity_scale = self.force_velocity_medium_scale
        else:
            self.force_velocity_scale = 1.0

        if not self.enable_high_force_hold:
            self.high_force_hold_start_time = None
            self.high_force_hold_active = False
            return

        now = float(self.data.time)
        release_threshold = (
            self.high_force_hold_threshold
            * self.high_force_hold_release_ratio
        )

        if self.high_force_hold_active:
            if force_norm < release_threshold:
                self.high_force_hold_active = False
                self.high_force_hold_start_time = None
                print(
                    "[HighForceHold] Released: "
                    f"force_norm={force_norm:.4f} N < "
                    f"{release_threshold:.4f} N"
                )
            return

        if force_norm < self.high_force_hold_threshold:
            self.high_force_hold_start_time = None
            return

        if self.high_force_hold_start_time is None:
            self.high_force_hold_start_time = now

        if (
            now - self.high_force_hold_start_time
            >= self.high_force_hold_dwell_time
        ):
            self.high_force_hold_active = True
            print(
                "[HighForceHold] Activated: "
                f"force_norm={force_norm:.4f} N, "
                f"dwell={self.high_force_hold_dwell_time:.4f} s"
            )

    def _step_command_toward_target_locked(self, dt: float):
        max_step = (
            self.max_joint_velocity
            * self.force_velocity_scale
            * dt
        )

        for i in range(min(len(self.command_joints), len(self.target_joints))):
            delta = self.target_joints[i] - self.command_joints[i]
            delta = max(-max_step, min(max_step, delta))
            self.command_joints[i] += delta

    def _apply_actuator_targets(self, target_joints: List[float]):
        for i, joint_name in enumerate(self.joint_names):
            actuator_id = self.actuator_map.get(joint_name)
            if actuator_id is not None and actuator_id >= 0:
                self.data.ctrl[actuator_id] = target_joints[i]

    def _physics_step(self):
        with self.lock:
            if not getattr(self, "terminal_hold_active", False):
                self._update_force_velocity_filter_locked()

                if self.high_force_hold_active:
                    q_now = self._get_current_arm_qpos_locked()
                    n = len(self.arm_joint_names)
                    self.target_joints[:n] = q_now.tolist()
                    self.command_joints[:n] = q_now.tolist()
                else:
                    self._step_command_toward_target_locked(
                        self.sim_timestep
                    )

            self._apply_actuator_targets(self.command_joints)
            mujoco.mj_step(self.model, self.data)

            # All live MjData reads stay inside the same ownership boundary as
            # mj_step, including alarms, task state and recorder snapshots.
            self._update_joint_torque_alarm_locked()
            self._update_ft_wrench_alarm_locked()
            self._update_task_success_auto_stop_locked()

            if self.data_recorder is not None:
                self.data_recorder.record_if_needed(self)

            recording_request = None
            if self.hdf5_recorder is not None:
                try:
                    recording_request = self.hdf5_recorder.record_if_needed(
                        self
                    )
                except Exception as e:
                    print(f"[HDF5Recorder] snapshot failed: {e}")

            self._schedule_render_snapshot_locked(recording_request)

    # ------------------------------------------------------------------
    # Task/debug helpers
    # ------------------------------------------------------------------
    def get_site_position(self, site_name: str) -> Optional[List[float]]:
        site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, site_name)
        if site_id == -1:
            return None
        with self.lock:
            mujoco.mj_forward(self.model, self.data)
            return self.data.site_xpos[site_id].copy().tolist()

    def get_body_position(self, body_name: str) -> Optional[List[float]]:
        body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, body_name)
        if body_id == -1:
            return None
        with self.lock:
            mujoco.mj_forward(self.model, self.data)
            return self.data.xpos[body_id].copy().tolist()

    def get_sensor_data(self, sensor_name: str) -> Optional[List[float]]:
        sensor_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SENSOR, sensor_name)
        if sensor_id == -1:
            return None

        adr = self.model.sensor_adr[sensor_id]
        dim = self.model.sensor_dim[sensor_id]
        with self.lock:
            return self.data.sensordata[adr:adr + dim].copy().tolist()

    def get_peg_ft_sensor(self) -> Optional[List[float]]:
        """
        Return [Fx, Fy, Fz, Tx, Ty, Tz] from MuJoCo force/torque sensors if present.
        The values are expressed in the ft_sensor_site frame.
        """
        f = self.get_sensor_data("peg_ft_force")
        t = self.get_sensor_data("peg_ft_torque")
        if f is None or t is None:
            return None
        return f + t

    def print_task_contacts(self, max_contacts: int = 10):
        """
        Print contact pairs involving the cylindrical peg.
        Useful for verifying that contact detection is active.
        """
        with self.lock:
            mujoco.mj_forward(self.model, self.data)
            n = min(self.data.ncon, max_contacts)
            for i in range(n):
                c = self.data.contact[i]
                g1 = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, c.geom1)
                g2 = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, c.geom2)
                if g1 == "cylindrical_peg" or g2 == "cylindrical_peg":
                    print(f"[contact {i}] {g1} <-> {g2}, dist={c.dist:.6f}")
    
    def _is_hdf5_recording_active(self) -> bool:
        """
        Return whether an HDF5 episode is currently being recorded.
        """
        return (
            self.hdf5_recorder is not None
            and bool(getattr(self.hdf5_recorder, "active", False))
        )
    
    def _apply_alarm_color_locked(self):
        """Apply peg color according to active alarms."""
        joint_active = bool(getattr(self, "joint_torque_alarm_active", False))
        ft_active = bool(getattr(self, "ft_wrench_alarm_active", False))

        if joint_active and ft_active:
            self.set_peg_color(self.both_alarm_peg_rgba)
        elif joint_active:
            self.set_peg_color(self.joint_torque_alarm_peg_rgba)
        elif ft_active:
            self.set_peg_color(self.ft_wrench_alarm_peg_rgba)
        else:
            self.set_peg_color(self.default_peg_rgba)
    
    def reset_ft_wrench_alarm(self):
        """Clear latched end-effector FT wrench alarm."""
        with self.lock:
            self.ft_wrench_alarm_active = False
            self.ft_wrench_alarm_start_time = None
            self.ft_wrench_alarm_first_time = None
            self.ft_wrench_alarm_first_force_norm = 0.0
            self.ft_wrench_alarm_first_torque_norm = 0.0
            self.ft_wrench_alarm_reason = ""
            self._apply_alarm_color_locked()


    def _make_quality_alarm_message(self) -> str:
        """Build quality warning message for service response."""
        msgs = []

        if getattr(self, "joint_torque_alarm_active", False):
            msgs.append(
                "joint_torque_alarm: "
                f"{self.joint_torque_alarm_first_joint}, "
                f"{self.joint_torque_alarm_first_value:.3f} Nm > "
                f"{self.joint_torque_alarm_first_limit:.3f} Nm"
            )

        if getattr(self, "ft_wrench_alarm_active", False):
            msgs.append(
                "ft_wrench_alarm: "
                f"force_norm={self.ft_wrench_alarm_first_force_norm:.3f} N, "
                f"torque_norm={self.ft_wrench_alarm_first_torque_norm:.3f} Nm, "
                f"{self.ft_wrench_alarm_reason}"
            )

        if not msgs:
            return ""

        return " Quality warning: " + " | ".join(msgs) + "."
    
    def _get_sensor_data_locked(self, sensor_name: str):
        if sensor_name not in self._sensor_address_cache:
            sensor_id = mujoco.mj_name2id(
                self.model,
                mujoco.mjtObj.mjOBJ_SENSOR,
                sensor_name,
            )
            self._sensor_address_cache[sensor_name] = (
                None
                if sensor_id == -1
                else (
                    int(self.model.sensor_adr[sensor_id]),
                    int(self.model.sensor_dim[sensor_id]),
                )
            )

        sensor_address = self._sensor_address_cache[sensor_name]
        if sensor_address is None:
            return None

        adr, dim = sensor_address
        return self.data.sensordata[adr:adr + dim].copy()


    def _get_peg_ft_sensor_locked(self):
        """Return [Fx, Fy, Fz, Tx, Ty, Tz] from MuJoCo FT sensors."""
        f = self._get_sensor_data_locked("peg_ft_force")
        t = self._get_sensor_data_locked("peg_ft_torque")

        if f is None or t is None:
            return None

        return np.concatenate([f, t], axis=0)
    
    def _update_ft_wrench_alarm_locked(self):
        """Check end-effector FT wrench limits and update peg color.

        Call inside self.lock, after mujoco.mj_step().
        """
        if not getattr(self, "enable_ft_wrench_alarm", True):
            return

        if (
            getattr(self, "ft_wrench_alarm_only_when_recording", True)
            and not self._is_hdf5_recording_active()
        ):
            return

        if (
            getattr(self, "ft_wrench_alarm_latched", True)
            and getattr(self, "ft_wrench_alarm_active", False)
        ):
            return

        wrench = self._get_peg_ft_sensor_locked()
        if wrench is None or len(wrench) < 6:
            return

        wrench = np.asarray(wrench, dtype=float).reshape(-1)
        force_norm = float(np.linalg.norm(wrench[:3]))
        torque_norm = float(np.linalg.norm(wrench[3:6]))

        force_over = force_norm > self.ft_force_norm_limit
        torque_over = torque_norm > self.ft_torque_norm_limit

        now = float(self.data.time)

        if force_over or torque_over:
            if self.ft_wrench_alarm_start_time is None:
                self.ft_wrench_alarm_start_time = now

            over_duration = now - self.ft_wrench_alarm_start_time

            if over_duration < self.ft_wrench_alarm_dwell_time:
                return

            reasons = []
            if force_over:
                reasons.append(
                    f"force_norm={force_norm:.3f}N > "
                    f"{self.ft_force_norm_limit:.3f}N"
                )
            if torque_over:
                reasons.append(
                    f"torque_norm={torque_norm:.3f}Nm > "
                    f"{self.ft_torque_norm_limit:.3f}Nm"
                )

            if not self.ft_wrench_alarm_active:
                self.ft_wrench_alarm_active = True
                self.ft_wrench_alarm_first_time = now
                self.ft_wrench_alarm_first_force_norm = force_norm
                self.ft_wrench_alarm_first_torque_norm = torque_norm
                self.ft_wrench_alarm_reason = "; ".join(reasons)

                print(
                    "[FTWrenchAlarm] Over limit: "
                    f"force_norm={force_norm:.4f} N, "
                    f"torque_norm={torque_norm:.4f} Nm, "
                    f"duration={over_duration:.4f} s, "
                    f"t={now:.4f}"
                )

                if self.hdf5_recorder is not None:
                    try:
                        if getattr(self.hdf5_recorder, "active", False):
                            self.hdf5_recorder.add_event("ft_wrench_over_limit")
                    except Exception as e:
                        print(f"[FTWrenchAlarm] Failed to add HDF5 event: {e}")

                self._apply_alarm_color_locked()

                if getattr(self, "ft_wrench_alarm_freeze_on_trigger", False):
                    q_now = self._get_current_arm_qpos_locked()
                    n = len(self.arm_joint_names)
                    self.target_joints[:n] = q_now.tolist()
                    self.command_joints[:n] = q_now.tolist()
                    self._apply_actuator_targets(self.command_joints)
                    self.accept_teleop_commands = False
                    print("[FTWrenchAlarm] Freeze teleop target at current qpos.")

            return

        # Below threshold.
        self.ft_wrench_alarm_start_time = None

        if not getattr(self, "ft_wrench_alarm_latched", True):
            if self.ft_wrench_alarm_active:
                self.ft_wrench_alarm_active = False
                self.ft_wrench_alarm_first_time = None
                self.ft_wrench_alarm_reason = ""
                self._apply_alarm_color_locked()


    def _get_arm_joint_torques_locked(self) -> np.ndarray:
        """
        Return actuator generalized torques for the arm joints.

        For hinge joints in MuJoCo, data.qfrc_actuator[dof_addr] is in Nm.
        The order follows self.arm_joint_names.
        """
        return self.data.qfrc_actuator[
            self._arm_joint_dof_addresses
        ].copy()


    def reset_joint_torque_alarm(self):
        """
        Clear latched joint torque alarm.

        This should be called before starting a new recording episode.
        """
        with self.lock:
            self.joint_torque_alarm_active = False
            self.joint_torque_alarm_first_time = None
            self.joint_torque_alarm_first_joint = ""
            self.joint_torque_alarm_first_value = 0.0
            self.joint_torque_alarm_first_limit = 0.0
            self.joint_torque_alarm_last_values = np.zeros(
                len(self.arm_joint_names),
                dtype=float,
            )

            # self.set_peg_color(self.default_peg_rgba)
            self._apply_alarm_color_locked()


    def _update_joint_torque_alarm_locked(self):
        """
        Check joint torque limits and update peg color.

        Call this inside self.lock, after mujoco.mj_step().
        """
        if not self.enable_joint_torque_alarm:
            return

        if (
            self.joint_torque_alarm_only_when_recording
            and not self._is_hdf5_recording_active()
        ):
            return

        torques = self._get_arm_joint_torques_locked()
        self.joint_torque_alarm_last_values = torques.copy()

        abs_torques = np.abs(torques)
        over_margin = abs_torques - self.joint_torque_limits

        # Ignore NaN joints.
        over_margin = np.where(np.isfinite(over_margin), over_margin, -np.inf)

        if not np.any(over_margin > 0.0):
            if not self.joint_torque_alarm_latched:
                self.joint_torque_alarm_active = False
                self.set_peg_color(self.default_peg_rgba)
            return

        first_idx = int(np.argmax(over_margin))

        if not self.joint_torque_alarm_active:
            self.joint_torque_alarm_active = True
            self.joint_torque_alarm_first_time = float(self.data.time)
            self.joint_torque_alarm_first_joint = self.arm_joint_names[first_idx]
            self.joint_torque_alarm_first_value = float(torques[first_idx])
            self.joint_torque_alarm_first_limit = float(
                self.joint_torque_limits[first_idx]
            )

            print(
                "[JointTorqueAlarm] Over limit: "
                f"joint={self.joint_torque_alarm_first_joint}, "
                f"torque={self.joint_torque_alarm_first_value:.4f} Nm, "
                f"limit={self.joint_torque_alarm_first_limit:.4f} Nm, "
                f"t={self.joint_torque_alarm_first_time:.4f}"
            )

            if self.hdf5_recorder is not None:
                try:
                    if getattr(self.hdf5_recorder, "active", False):
                        self.hdf5_recorder.add_event("joint_torque_over_limit")
                except Exception as e:
                    print(f"[JointTorqueAlarm] Failed to add HDF5 event: {e}")

        # Highest-priority visual alarm.
        # self.set_peg_color(self.joint_torque_alarm_peg_rgba)
        self._apply_alarm_color_locked()
    
    def _reset_task_success_state_locked(self):
        """
        Reset task-success and terminal-hold state for a new episode.
        Call inside self.lock.
        """
        self.task_success_accumulated_time = 0.0
        self.task_success_last_check_time = None
        self.task_success_triggered = False

        self.terminal_hold_active = False
        self.terminal_hold_start_time = None
        self.terminal_hold_command_start = None
        self.terminal_hold_command_goal = None
        self.terminal_hold_stop_started = False

        self.pending_auto_completed_review = False
        self.pending_auto_completed_episode_path = ""
        self.pending_auto_completed_episode_dir = ""
        self._reset_force_velocity_filter_locked()


    def _get_site_distance_locked(self, site_a_id: int, site_b_id: int) -> float:
        if site_a_id == -1 or site_b_id == -1:
            return float("inf")

        pa = self.data.site_xpos[site_a_id]
        pb = self.data.site_xpos[site_b_id]

        return float(np.linalg.norm(pa - pb))


    def _get_current_arm_qpos_locked(self) -> np.ndarray:
        """
        Current actual qpos of arm joints in internal MuJoCo sign convention.
        Order follows self.arm_joint_names.
        """
        q = np.zeros(len(self.arm_joint_names), dtype=float)

        for i, joint_name in enumerate(self.arm_joint_names):
            joint_id = mujoco.mj_name2id(
                self.model,
                mujoco.mjtObj.mjOBJ_JOINT,
                joint_name,
            )

            if joint_id == -1:
                continue

            qpos_addr = int(self.model.jnt_qposadr[joint_id])

            if 0 <= qpos_addr < self.data.qpos.shape[0]:
                q[i] = float(self.data.qpos[qpos_addr])

        return q


    def _start_terminal_hold_locked(self, distance: float, now: float):
        """
        Enter terminal hold:
        - block new teleop commands
        - blend command_joints to current qpos
        - keep recording for terminal hold duration
        """
        if self.terminal_hold_active:
            return

        self.task_success_triggered = True
        self.terminal_hold_active = True
        self.terminal_hold_start_time = float(now)
        self.terminal_hold_stop_started = False

        if self.task_success_stop_accepting_teleop:
            self.accept_teleop_commands = False

        qpos_now = self._get_current_arm_qpos_locked()

        self.terminal_hold_command_start = np.asarray(
            self.command_joints[:len(self.arm_joint_names)],
            dtype=float,
        ).copy()

        self.terminal_hold_command_goal = qpos_now.copy()

        print(
            "[TaskSuccessAutoStop] Site success reached. "
            f"distance={distance:.6f} m, "
            f"t={now:.3f}. Enter terminal hold."
        )

        if self.hdf5_recorder is not None:
            try:
                if getattr(self.hdf5_recorder, "active", False):
                    self.hdf5_recorder.add_event("task_success_site_reached")
                    self.hdf5_recorder.add_event("terminal_hold_start")
            except Exception as e:
                print(f"[TaskSuccessAutoStop] Failed to add HDF5 event: {e}")


    def _update_terminal_hold_locked(self, now: float):
        """
        During terminal hold, overwrite target/command with a safe hold command.
        The command blends from previous command to current qpos, then stays there.
        """
        if not self.terminal_hold_active:
            return

        if self.terminal_hold_start_time is None:
            self.terminal_hold_start_time = float(now)

        elapsed = float(now - self.terminal_hold_start_time)

        blend_time = max(self.task_success_blend_to_qpos_time, 1e-6)
        alpha = min(1.0, max(0.0, elapsed / blend_time))

        q_start = self.terminal_hold_command_start
        q_goal = self.terminal_hold_command_goal

        if q_start is None or q_goal is None:
            q_goal = self._get_current_arm_qpos_locked()
            q_start = q_goal.copy()
            self.terminal_hold_command_start = q_start
            self.terminal_hold_command_goal = q_goal

        hold_cmd = (1.0 - alpha) * q_start + alpha * q_goal

        n = len(self.arm_joint_names)
        self.target_joints[:n] = hold_cmd.tolist()
        self.command_joints[:n] = hold_cmd.tolist()

        self._apply_actuator_targets(self.command_joints)

        if elapsed >= self.task_success_terminal_hold_time:
            if not self.terminal_hold_stop_started:
                self.terminal_hold_stop_started = True

                if self.task_success_auto_stop_recording:
                    print(
                        "[TaskSuccessAutoStop] Terminal hold finished. "
                        "Auto-stopping HDF5 recording."
                    )

                    threading.Thread(
                        target=self._auto_stop_recording_for_task_success,
                        daemon=True,
                    ).start()
                else:
                    print(
                        "[TaskSuccessAutoStop] Terminal hold finished. "
                        "External scripted lifecycle will stop recording."
                    )


    def _update_task_success_auto_stop_locked(self):
        """
        Check site-distance success condition and update terminal hold.
        Call inside self.lock after mujoco.mj_step().
        """
        if not self.enable_task_success_auto_stop:
            return

        now = float(self.data.time)

        if self.terminal_hold_active:
            self._update_terminal_hold_locked(now)
            return

        if self.task_success_triggered:
            return

        if (
            self.task_success_only_when_recording
            and not self._is_hdf5_recording_active()
        ):
            self.task_success_accumulated_time = 0.0
            self.task_success_last_check_time = now
            return

        if self.task_success_last_check_time is None:
            dt = float(self.model.opt.timestep)
        else:
            dt = max(0.0, now - self.task_success_last_check_time)

        self.task_success_last_check_time = now

        dist = self._get_site_distance_locked(
            self.task_success_peg_site_id,
            self.task_success_hole_site_id,
        )

        if dist <= self.task_success_distance:
            self.task_success_accumulated_time += dt
        else:
            self.task_success_accumulated_time = 0.0
            return

        if self.task_success_accumulated_time < self.task_success_dwell_time:
            return

        self._start_terminal_hold_locked(distance=dist, now=now)
    
    def set_peg_color(self, rgba):
        """
        动态修改 peg 的显示颜色。

        优先修改 material 颜色，因为 cylindrical_peg 使用了 material="mat_peg"。
        同时也修改 geom_rgba，避免不同 viewer/渲染路径下不生效。
        """
        rgba = np.asarray(rgba, dtype=float)

        if rgba.shape[0] != 4:
            return

        if getattr(self, "peg_mat_id", -1) != -1:
            self.model.mat_rgba[self.peg_mat_id] = rgba

        if getattr(self, "peg_geom_id", -1) != -1:
            self.model.geom_rgba[self.peg_geom_id] = rgba

    def _get_site_xpos_for_viewer(self, site_name: str):
        """
        获取 site 的世界坐标。
        这个函数只在仿真线程里绘图用，不额外加锁，避免和 mj_step/viewer.sync 互相阻塞。
        """
        site_id = mujoco.mj_name2id(
            self.model,
            mujoco.mjtObj.mjOBJ_SITE,
            site_name,
        )

        if site_id == -1:
            return None

        return self.data.site_xpos[site_id].copy()


    def _add_connector_geom(self, viewer, geom_type, start, end, width, rgba):
        """
        在 MuJoCo viewer.user_scn 里添加一条 connector。
        可以是 LINE / ARROW / CAPSULE 等。
        """
        start = np.asarray(start, dtype=float)
        end = np.asarray(end, dtype=float)
        rgba = np.asarray(rgba, dtype=float)

        if np.linalg.norm(end - start) < 1e-6:
            return

        scene = viewer.user_scn

        if scene.ngeom >= len(scene.geoms):
            return

        geom = scene.geoms[scene.ngeom]

        mujoco.mjv_initGeom(
            geom,
            geom_type,
            np.zeros(3),
            np.zeros(3),
            np.eye(3).reshape(-1),
            rgba,
        )

        mujoco.mjv_connector(
            geom,
            geom_type,
            width,
            start,
            end,
        )

        scene.ngeom += 1
    

    def _apply_hole_position(self, sample: Dict[str, Any]) -> Dict[str, Any]:
        """Apply one prepared task offset and expose it as episode metadata."""
        offset = np.asarray(sample["hole_actual_offset_xyz"], dtype=float)
        if offset.shape != (3,) or not np.all(np.isfinite(offset)):
            raise ValueError(f"Invalid hole offset: {offset}")

        body_pos = self.hole_nominal_body_pos + offset
        if self.hole_body_id != -1:
            with self.lock:
                self.model.body_pos[self.hole_body_id] = body_pos
                mujoco.mj_forward(self.model, self.data)

        context = dict(sample)
        context.update(
            {
                "enabled": self.hole_sampling_mode != "fixed",
                "hole_body_name": self.hole_body_name,
                "hole_nominal_body_pos": self.hole_nominal_body_pos.tolist(),
                "hole_actual_body_pos": body_pos.tolist(),
                "hole_offset_from_nominal_xyz": offset.tolist(),
            }
        )
        self.current_hole_sample = context
        self.last_hole_randomization = dict(context)

        cell = context.get("hole_grid_cell_label", "n/a")
        cycle = context.get("hole_grid_cycle", "n/a")
        index = context.get("hole_grid_index", "n/a")
        print(
            "[HolePosition] "
            f"mode={self.hole_sampling_mode}, cycle={cycle}, "
            f"index={index}, cell={cell}, offset={offset.tolist()}, "
            f"body_pos={body_pos.tolist()}"
        )
        return dict(context)

    def prepare_hole_for_episode(self) -> Dict[str, Any]:
        """Prepare the current grid cell (or a compatibility sampling mode)."""
        if self.hole_sampling_mode == "grid":
            if self.hole_grid_scheduler is None:
                raise RuntimeError("Grid sampling enabled without a scheduler")
            sample = self.hole_grid_scheduler.current()
        elif self.hole_sampling_mode == "uniform_random":
            offset = [
                float(self.hole_rng.uniform(*self.hole_random_x_range)),
                float(self.hole_rng.uniform(*self.hole_random_y_range)),
                float(self.hole_rng.uniform(*self.hole_random_z_range)),
            ]
            sample = {
                "hole_sampling_mode": "uniform_random",
                "hole_random_seed": (
                    -1 if self.hole_random_seed is None else int(self.hole_random_seed)
                ),
                "hole_actual_offset_xyz": offset,
            }
        else:
            sample = {
                "hole_sampling_mode": "fixed",
                "hole_actual_offset_xyz": [0.0, 0.0, 0.0],
            }

        return self._apply_hole_position(sample)

    def finalize_hole_after_episode(self, keep: bool) -> Dict[str, Any]:
        """Advance only when policy permits, then prepare the next episode."""
        should_advance = bool(keep) or self.hole_grid_advance_policy == "on_attempt"

        if self.hole_sampling_mode == "grid":
            self.hole_grid_scheduler.complete_episode(
                keep=bool(keep),
                advance_policy=self.hole_grid_advance_policy,
            )
            return self.prepare_hole_for_episode()

        if self.hole_sampling_mode == "uniform_random" and should_advance:
            return self.prepare_hole_for_episode()

        # A discarded grid/uniform episode intentionally retries the same pose.
        return self._apply_hole_position(self.current_hole_sample)

    def randomize_hole_position(self):
        """Compatibility wrapper for legacy callers."""
        if self.hole_sampling_mode == "grid" and self.hole_grid_scheduler is not None:
            self.hole_grid_scheduler.advance()
        return self.prepare_hole_for_episode()


    def draw_insertion_guides(self, viewer):
        """
        绘制插孔视觉引导：
        1. peg_tip -> hole_entrance 的动态箭头，颜色随对准误差变化；
        2. 洞口法向/插入方向黄色箭头。
        """
        if not self.enable_visual_guides:
            self.set_peg_color(self.default_peg_rgba)
            return

        # 清空上一帧自定义视觉几何。
        # 如果后面还想画别的自定义元素，也统一放在这个函数里画。
        viewer.user_scn.ngeom = 0

        peg_tip = self._get_site_xpos_for_viewer("peg_tip_site")
        hole_center = self._get_site_xpos_for_viewer("hole_center_site")

        if peg_tip is None or hole_center is None:
            self.set_peg_color(self.default_peg_rgba)
            return

        axis = self.hole_axis_world

        # 洞口可视入口点。
        # 如果发现箭头/marker 在墙背面，把 hole_axis_world 改成 [0, 1, 0]。
        hole_entrance = hole_center + axis * self.hole_entrance_offset

        # 从 peg tip 指向洞口入口的误差向量
        error_vec = hole_entrance - peg_tip

        # 将误差分解为：
        #   insertion_error：沿插入轴方向的误差
        #   lateral_error：垂直于插入轴的对准误差
        insertion_error = float(np.dot(error_vec, axis))
        lateral_vec = error_vec - insertion_error * axis
        lateral_error = float(np.linalg.norm(lateral_vec))

        if lateral_error <= self.guide_green_threshold:
            # 对准较好：绿色
            guide_rgba = [0.0, 1.0, 0.0, 0.85]
            peg_rgba = [0.0, 0.95, 0.10, 1.0]
        elif lateral_error <= self.guide_yellow_threshold:
            # 中等偏差：黄色
            guide_rgba = [1.0, 1.0, 0.0, 0.85]
            peg_rgba = [1.0, 0.85, 0.0, 1.0]
        else:
            # 偏差较大：红色
            guide_rgba = [1.0, 0.0, 0.0, 0.85]
            peg_rgba = [1.0, 0.10, 0.05, 1.0]

        self.set_peg_color(peg_rgba)

        # 1. peg_tip -> hole_entrance 动态引导箭头
        self._add_connector_geom(
            viewer=viewer,
            geom_type=mujoco.mjtGeom.mjGEOM_ARROW,
            start=peg_tip,
            end=hole_entrance,
            width=self.guide_arrow_width,
            rgba=guide_rgba,
        )

        # 2. 洞口法向 / 插入方向箭头
        # axis 指向洞外侧，所以正确插入方向是 -axis。
        normal_start = hole_entrance
        normal_end = hole_entrance - axis * self.hole_axis_arrow_length

        self._add_connector_geom(
            viewer=viewer,
            geom_type=mujoco.mjtGeom.mjGEOM_ARROW,
            start=normal_start,
            end=normal_end,
            width=0.005,
            rgba=[1.0, 0.85, 0.0, 0.90],
        )
    

    def set_viewer_fixed_camera(self, viewer, camera_name: str):
        """
        强制将 MuJoCo viewer 切换到指定 fixed camera。
        """
        cam_id = mujoco.mj_name2id(
            self.model,
            mujoco.mjtObj.mjOBJ_CAMERA,
            camera_name
        )

        if cam_id == -1:
            print(f"[Camera] Cannot find camera: {camera_name}")
            return

        with viewer.lock():
            viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
            viewer.cam.fixedcamid = cam_id

        print(f"[Camera] Switched viewer to fixed camera: {camera_name}")

    # ------------------------------------------------------------------
    # Simulation / visualization thread
    # ------------------------------------------------------------------
    def visualization_thread(self):
        print("启动可视化/物理仿真线程...")

        # Reset and restore initial pose/ctrl.
        with self.lock:
            q0 = list(self.command_joints)
            mujoco.mj_resetData(self.model, self.data)
            self._hard_set_qpos(q0)
            self._apply_actuator_targets(q0)
            mujoco.mj_forward(self.model, self.data)

        try:
            if self.launch_viewer:
                with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
                    self.viewer_handle = viewer
                    print("可视化已启动，按 ESC 或关闭窗口退出")
                    self.set_viewer_fixed_camera(viewer, self.cctv_camera)
                    self.viewer_running = True
                    last_sync = time.perf_counter()
                    next_step_deadline = last_sync

                    while (
                        viewer.is_running()
                        and self.running
                        and not self.stop_event.is_set()
                    ):
                        if self.control_mode == "actuator":
                            self._physics_step()
                        else:
                            with self.lock:
                                self.update_positions(self.target_joints)

                        now = time.perf_counter()
                        if now - last_sync >= self.viewer_period:
                            with self.lock:
                                with viewer.lock():
                                    if self.enable_visual_guides:
                                        self.draw_insertion_guides(viewer)
                                viewer.sync()
                            last_sync = now

                        if self.realtime:
                            next_step_deadline += self.sim_timestep
                            sleep_time = next_step_deadline - time.perf_counter()
                            if sleep_time > 0:
                                time.sleep(sleep_time)
                            else:
                                lag = -sleep_time
                                self.physics_overrun_count += 1
                                self.physics_max_lag = max(
                                    self.physics_max_lag, lag
                                )
                                if lag > 0.1:
                                    next_step_deadline = time.perf_counter()
                        self.physics_step_count += 1

                    self.viewer_running = False
                    self.viewer_handle = None
                    print("可视化窗口关闭，仿真结束")
            else:
                next_step_deadline = time.perf_counter()
                while self.running and not self.stop_event.is_set():
                    if self.control_mode == "actuator":
                        self._physics_step()
                    else:
                        with self.lock:
                            self.update_positions(self.target_joints)

                    if self.realtime:
                        next_step_deadline += self.sim_timestep
                        sleep_time = next_step_deadline - time.perf_counter()
                        if sleep_time > 0:
                            time.sleep(sleep_time)
                        else:
                            lag = -sleep_time
                            self.physics_overrun_count += 1
                            self.physics_max_lag = max(
                                self.physics_max_lag, lag
                            )
                            if lag > 0.1:
                                next_step_deadline = time.perf_counter()
                    self.physics_step_count += 1

        except Exception as e:
            print(f"可视化/仿真错误: {e}")
            import traceback
            traceback.print_exc()
            self.viewer_running = False
            self.viewer_handle = None
        finally:
            self.running = False
            self.stop_event.set()
            self.render_stop_event.set()

    def start_simulation(self):
        if self.running:
            print("仿真已经在运行")
            return

        self.stop_event.clear()
        self._start_render_worker()
        self.running = True
        self.vis_thread = threading.Thread(target=self.visualization_thread, daemon=True)
        self.vis_thread.start()

        if self.launch_viewer and self.viewer_start_wait > 0.0:
            time.sleep(self.viewer_start_wait)
        print("仿真已启动，按 Ctrl+C 停止主程序...")

    def disconnect(self):
        self.running = False
        self.stop_event.set()
        viewer = self.viewer_handle
        if viewer is not None:
            try:
                viewer.close()
            except Exception as exc:
                print(f"[Shutdown] Viewer close failed: {exc}")
        if self.vis_thread is not None and self.vis_thread.is_alive():
            self.vis_thread.join(timeout=10.0)
        if self.vis_thread is not None and self.vis_thread.is_alive():
            raise RuntimeError(
                "Simulation thread did not stop; recorder/render resources "
                "were left open to avoid cross-thread use-after-close"
            )

        # Keep the render worker alive while the recorder drains pending image
        # requests, then let the worker close its own OpenGL/OpenCV resources.
        if self.hdf5_recorder is not None:
            with self.lock:
                self.hdf5_recorder.close()

        self.render_stop_event.set()
        if self.render_thread is not None and self.render_thread.is_alive():
            self.render_thread.join(timeout=10.0)

        # Stop and close data recorder
        if self.data_recorder is not None:
            self.data_recorder.close()

        print(
            "[RuntimeMetrics] "
            f"physics_steps={self.physics_step_count}, "
            f"overruns={self.physics_overrun_count}, "
            f"max_lag={self.physics_max_lag:.6f}s, "
            f"render_drops={self.render_snapshot_drops}, "
            f"render_errors={self.render_error_count}, "
            f"video_errors={self.cctv_frame_sink_error_count}"
        )
        print("仿真停止")


# Compatibility alias.
# If any older code imports RobotControllerMuJoCo from this file, it will still work.
RobotControllerMuJoCo = RobotControllerMuJoCoPegTool


if __name__ == "__main__":
    config = {
        "control_mode": "actuator",
        "launch_viewer": True,
        "auto_start": True,
        "viewer_rate": 60,
        "realtime": True,
        "max_joint_velocity": 0.5,
        "initial_arm_joints": [-0.046, -0.2, 0.0, 1.6, -1.32, 0.005, 0.005],
        "enable_visual_guides": False,

        # 当前 wall-parallel 版本默认插入方向沿 y 轴。
        # 如果黄色箭头方向反了，把它改成 [0.0, 1.0, 0.0]。
        "hole_axis_world": [0.0, 1.0, 0.0],

        # 洞口可视入口点相对 hole_center 的偏移
        "hole_entrance_offset": 0.026,

        # 黄色插入方向箭头长度
        "hole_axis_arrow_length": 0.10,

        # peg->hole 引导箭头粗细
        "guide_arrow_width": 0.003,

        # 对准误差颜色阈值，单位 m
        "guide_green_threshold": 0.010,
        "guide_yellow_threshold": 0.030,

        # video stream config
        "show_camera_streams": True,
        "camera_stream_width": 640,
        "camera_stream_height": 480,
        "camera_stream_fps": 15.0,

        "monitor_camera_names": ["ee_cam", "base_top_cam"],
    }

    try:
        print("创建仿真器...")
        model_path = "/home/stw/pangu/src/arm_teleop/model/pangu_all_right.xml"
        simulator = RobotControllerMuJoCoPegTool(model_path, config)

        while simulator.running:
            time.sleep(1.0)

        print("仿真完成")

    except KeyboardInterrupt:
        print("收到 Ctrl+C，正在退出...")
        try:
            simulator.disconnect()
        except NameError:
            pass
    except Exception as e:
        print(f"仿真器错误: {e}")
        import traceback
        traceback.print_exc()
