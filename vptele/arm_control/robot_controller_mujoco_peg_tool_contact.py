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

import time
import threading
from typing import List, Optional, Dict, Any

import numpy as np
import cv2
import mujoco
import mujoco.viewer
import h5py
import shutil

from utils.mujoco_data_recorder import MujocoDataRecorder
from vptele.utils.mujoco_hdf5_recorder import MujocoHDF5Recorder

import rospy
from arm_teleop.srv import SetRecording, SetRecordingResponse
from std_srvs.srv import Trigger, TriggerResponse


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
        self.last_camera_stream_time = 0.0

        self.monitor_camera_names = self.config.get(
            "monitor_camera_names",
            ["ee_cam", "base_top_cam"]
        )

        self.camera_renderer = None
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
            )

            if bool(self.config.get("hdf5_auto_start", False)):
                self.hdf5_recorder.start_episode(
                    label=self.config.get("hdf5_episode_label", "teleop")
                )


        

        # ############################################################# #
        # ---------------- ROS Data Recording Service ----------------- #
        # ############################################################# #
        self.recording_service = None

        self.enable_recording_service = bool(
            self.config.get("enable_recording_service", True)
        )

        self.recording_service_name = self.config.get(
            "recording_service_name",
            "/mujoco_hdf5_recording/set_recording"
        )

        if self.enable_recording_service and self.hdf5_recorder is not None:
            self.recording_service = rospy.Service(
                self.recording_service_name,
                SetRecording,
                self._handle_recording_service,
            )

            print(f"[Recording Service] Ready: {self.recording_service_name}")





        # ############################################################ #
        # ---------------- Target smoothing / safety ----------------- #
        # ############################################################ #

        self.max_joint_velocity = float(self.config.get("max_joint_velocity", 0.5))  # rad/s
        self.max_joint_step_qpos = float(self.config.get("max_joint_step_qpos", 0.015))  # rad/frame for qpos debug mode




        # ############################################################
        # # ---------------- Joint torque alarm ----------------------
        # ############################################################
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





        # ######################################################### #
        # -------------------- Hole randomization ----------------- #
        # ######################################################### #

        self.enable_hole_randomization = bool(
            self.config.get("enable_hole_randomization", False)
        )

        self.randomize_hole_on_record_start = bool(
            self.config.get("randomize_hole_on_record_start", True)
        )

        self.hole_random_body_name = self.config.get(
            "hole_random_body_name",
            "wall_task",
        )

        self.hole_random_body_id = mujoco.mj_name2id(
            self.model,
            mujoco.mjtObj.mjOBJ_BODY,
            self.hole_random_body_name,
        )

        if self.enable_hole_randomization and self.hole_random_body_id == -1:
            print(
                f"[Hole Randomization] Cannot find body: "
                f"{self.hole_random_body_name}. Disable randomization."
            )
            self.enable_hole_randomization = False

        if self.hole_random_body_id != -1:
            self.hole_nominal_body_pos = self.model.body_pos[
                self.hole_random_body_id
            ].copy()
        else:
            self.hole_nominal_body_pos = np.zeros(3, dtype=float)

        self.hole_random_x_range = self.config.get(
            "hole_random_x_range",
            [-0.01, 0.01],
        )
        self.hole_random_y_range = self.config.get(
            "hole_random_y_range",
            [0.0, 0.0],
        )
        self.hole_random_z_range = self.config.get(
            "hole_random_z_range",
            [-0.01, 0.01],
        )

        self.hole_random_seed = self.config.get("hole_random_seed", None)
        self.hole_rng = np.random.default_rng(self.hole_random_seed)

        self.last_hole_randomization = {
            "enabled": False,
            "body_name": self.hole_random_body_name,
            "nominal_body_pos": self.hole_nominal_body_pos.tolist(),
            "offset_xyz": [0.0, 0.0, 0.0],
            "body_pos": self.hole_nominal_body_pos.tolist(),
        }





        # #################################### #
        # ----- Launching Initialization ----- #
        # #################################### #

        self.launch_viewer = bool(self.config.get("launch_viewer", True))
        self.viewer_start_wait = float(self.config.get("viewer_start_wait", 1.0))

        self.running = False
        self.viewer_running = False
        self.lock = threading.RLock()
        self.vis_thread: Optional[threading.Thread] = None

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

        if self.config.get("auto_start", True):
            print("启动 MuJoCo 仿真线程...")
            self.start_simulation()
    

    def _call_teleop_trigger_service(self, service_name: str, description: str) -> bool:
        """
        Call an arm_teleop Trigger service.

        Used to stop/recalibrate/start arm teleoperation around each recording episode.
        """
        if not getattr(self, "teleop_controlled_by_recording", True):
            return True

        try:
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

    
    def _handle_recording_service(self, req):
        """
        ROS service callback for starting/stopping HDF5 episode recording.

        req.record = True:
            start a new episode

        req.record = False:
            stop current episode
            if req.keep is False, delete the generated episode folder
        """
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

                # 3. 每条 episode 开始前随机孔洞位置
                if getattr(self, "randomize_hole_on_record_start", False):
                    self.randomize_hole_position()

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
                hdf5_path = self.hdf5_recorder.start_episode(label=label)

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
                        self.hdf5_recorder.stop_episode(status="teleop_start_failed")

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

                # 0. 立刻拒绝新的遥操作命令
                self.accept_teleop_commands = False

                # 1. 停止遥操作线程
                self._call_teleop_trigger_service(
                    self.teleop_stop_service_name,
                    "stop teleoperation before record stop",
                )

                # 2. 停止 HDF5 记录

                torque_alarm_msg = ""
                if getattr(self, "joint_torque_alarm_active", False):
                    torque_alarm_msg = (
                        " Joint torque alarm was triggered: "
                        f"{self.joint_torque_alarm_first_joint}, "
                        f"{self.joint_torque_alarm_first_value:.3f} Nm > "
                        f"{self.joint_torque_alarm_first_limit:.3f} Nm."
                    )

                hdf5_path = self.hdf5_recorder.stop_episode(
                    status="manual_keep" if req.keep else "manual_discard"
                )

                episode_path = str(hdf5_path) if hdf5_path is not None else ""

                # 3. reset 机械臂回默认位置。这个动作不进入 HDF5 数据。
                if getattr(self, "reset_arm_on_record_stop", True):
                    self.reset_arm_to_initial_pose()

                # 4. 不保留则删除 episode 文件夹
                if not req.keep:
                    if hdf5_path is not None:
                        episode_dir = hdf5_path.parent
                        if episode_dir.exists():
                            shutil.rmtree(episode_dir)

                    return SetRecordingResponse(
                        success=True,
                        active=False,
                        # message="Stopped recording, stopped teleoperation, reset arm, and discarded this episode.",
                        message=(
                            "Stopped recording, stopped teleoperation, reset arm, "
                            "and discarded this episode."
                            + torque_alarm_msg
                        ),
                        episode_path=episode_path,
                    )

                return SetRecordingResponse(
                    success=True,
                    active=False,
                    # message="Stopped recording, stopped teleoperation, reset arm, and kept this episode.",
                    message=(
                        "Stopped recording, stopped teleoperation, reset arm, "
                        "and kept this episode."
                        + torque_alarm_msg
                    ),
                    episode_path=episode_path,
                )
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
    def _ensure_camera_renderer(self):
        if self.camera_renderer is None:
            self.camera_renderer = mujoco.Renderer(
                self.model,
                height=self.camera_stream_height,
                width=self.camera_stream_width,
            )
    
    def _render_camera_rgb(self, camera_name: str):
        """
        渲染指定 MuJoCo camera，返回 RGB 图像。
        """
        if camera_name not in self.monitor_camera_ids:
            return None

        self._ensure_camera_renderer()

        cam_id = self.monitor_camera_ids[camera_name]
        self.camera_renderer.update_scene(self.data, camera=cam_id)
        rgb = self.camera_renderer.render()
        return rgb

    def update_camera_stream_windows(self):
        """
        显示两个相机的视频流：
        左边 ee_cam，右边 base_top_cam。
        """
        if not self.show_camera_streams:
            return

        now = time.perf_counter()
        if now - self.last_camera_stream_time < self.camera_stream_period:
            return

        frames = []
        labels = []

        for cam_name in self.monitor_camera_names:
            rgb = self._render_camera_rgb(cam_name)
            if rgb is None:
                continue

            # MuJoCo 返回 RGB，OpenCV 显示用 BGR
            bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

            # 在左上角打上相机名称
            cv2.putText(
                bgr,
                cam_name,
                (15, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.0,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )

            frames.append(bgr)
            labels.append(cam_name)

        if len(frames) == 0:
            return

        # 如果只有一个相机，就单独显示
        if len(frames) == 1:
            panel = frames[0]
        else:
            # 横向拼接两个画面
            panel = np.hstack(frames)

        cv2.imshow("Task Camera Streams", panel)
        cv2.waitKey(1)

        self.last_camera_stream_time = now

    

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
    
    def set_arm_positions(self, arm_target_joints: List[float]):
        """
        Public teleoperation interface.
        """
        if not getattr(self, "accept_teleop_commands", True):
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

    def _step_command_toward_target_locked(self, dt: float):
        max_step = self.max_joint_velocity * dt

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
            self._step_command_toward_target_locked(self.sim_timestep)
            self._apply_actuator_targets(self.command_joints)

        mujoco.mj_step(self.model, self.data)

        # Online data-quality alarm.
        # If any joint torque exceeds its limit during recording,
        # turn the peg green and latch the alarm.
        self._update_joint_torque_alarm_locked()


        # Data Recording
        if self.data_recorder is not None:
            self.data_recorder.record_if_needed(self)

        # if self.hdf5_recorder is not None:
        #     self.hdf5_recorder.record_if_needed(self)
        if self.hdf5_recorder is not None:
            try:
                self.hdf5_recorder.record_if_needed(self)
            except Exception as e:
                print(f"[HDF5Recorder] record_if_needed failed: {e}")
                try:
                    self.hdf5_recorder.active = False
                except Exception:
                    pass

        # Camera Streaming
        if self.show_camera_streams:
            self.update_camera_stream_windows()

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


    def _get_arm_joint_torques_locked(self) -> np.ndarray:
        """
        Return actuator generalized torques for the arm joints.

        For hinge joints in MuJoCo, data.qfrc_actuator[dof_addr] is in Nm.
        The order follows self.arm_joint_names.
        """
        torques = np.full(len(self.arm_joint_names), np.nan, dtype=float)

        for i, joint_name in enumerate(self.arm_joint_names):
            joint_id = mujoco.mj_name2id(
                self.model,
                mujoco.mjtObj.mjOBJ_JOINT,
                joint_name,
            )

            if joint_id == -1:
                continue

            dof_addr = int(self.model.jnt_dofadr[joint_id])

            if 0 <= dof_addr < self.data.qfrc_actuator.shape[0]:
                torques[i] = float(self.data.qfrc_actuator[dof_addr])

        return torques


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

            self.set_peg_color(self.default_peg_rgba)


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
        self.set_peg_color(self.joint_torque_alarm_peg_rgba)
    
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
    

    def randomize_hole_position(self):
        """
        Randomize the wall_task body position once before starting a new episode.

        For current pangu_all_right.xml:
            body name = wall_task
            nominal pos = [-0.25, -0.5, 1.0]

        Recommended:
            randomize x and z only;
            keep y fixed because y is insertion-depth direction.
        """
        if not self.enable_hole_randomization:
            return self.last_hole_randomization

        if self.hole_random_body_id == -1:
            return self.last_hole_randomization

        dx = float(
            self.hole_rng.uniform(
                float(self.hole_random_x_range[0]),
                float(self.hole_random_x_range[1]),
            )
        )
        dy = float(
            self.hole_rng.uniform(
                float(self.hole_random_y_range[0]),
                float(self.hole_random_y_range[1]),
            )
        )
        dz = float(
            self.hole_rng.uniform(
                float(self.hole_random_z_range[0]),
                float(self.hole_random_z_range[1]),
            )
        )

        offset = np.array([dx, dy, dz], dtype=float)
        new_pos = self.hole_nominal_body_pos + offset

        with self.lock:
            self.model.body_pos[self.hole_random_body_id] = new_pos
            mujoco.mj_forward(self.model, self.data)

        self.last_hole_randomization = {
            "enabled": True,
            "body_name": self.hole_random_body_name,
            "nominal_body_pos": self.hole_nominal_body_pos.tolist(),
            "offset_xyz": offset.tolist(),
            "body_pos": new_pos.tolist(),
        }

        print(
            "[Hole Randomization] "
            f"body={self.hole_random_body_name}, "
            f"offset={offset.tolist()}, "
            f"new_pos={new_pos.tolist()}"
        )

        return self.last_hole_randomization


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
                    print("可视化已启动，按 ESC 或关闭窗口退出")
                    self.set_viewer_fixed_camera(viewer, self.cctv_camera)
                    self.viewer_running = True
                    last_sync = time.perf_counter()

                    while viewer.is_running() and self.running:
                        step_start = time.perf_counter()

                        if self.control_mode == "actuator":
                            self._physics_step()
                        else:
                            with self.lock:
                                self.update_positions(self.target_joints)

                        now = time.perf_counter()
                        if now - last_sync >= self.viewer_period:
                            with self.lock:
                                if self.enable_visual_guides:
                                    self.draw_insertion_guides(viewer)
                            viewer.sync()
                            last_sync = now

                        if self.realtime:
                            elapsed = time.perf_counter() - step_start
                            sleep_time = self.sim_timestep - elapsed
                            if sleep_time > 0:
                                time.sleep(sleep_time)

                    self.viewer_running = False
                    print("可视化窗口关闭，仿真结束")
            else:
                while self.running:
                    step_start = time.perf_counter()

                    if self.control_mode == "actuator":
                        self._physics_step()
                    else:
                        with self.lock:
                            self.update_positions(self.target_joints)

                    if self.realtime:
                        elapsed = time.perf_counter() - step_start
                        sleep_time = self.sim_timestep - elapsed
                        if sleep_time > 0:
                            time.sleep(sleep_time)

        except Exception as e:
            print(f"可视化/仿真错误: {e}")
            import traceback
            traceback.print_exc()
            self.viewer_running = False

    def start_simulation(self):
        if self.running:
            print("仿真已经在运行")
            return

        self.running = True
        self.vis_thread = threading.Thread(target=self.visualization_thread, daemon=True)
        self.vis_thread.start()

        time.sleep(self.viewer_start_wait)
        print("仿真已启动，按 Ctrl+C 停止主程序...")

    def disconnect(self):
        self.running = False
        if self.vis_thread is not None and self.vis_thread.is_alive():
            self.vis_thread.join(timeout=1.0)

        # Stop and close data recorder
        if self.data_recorder is not None:
            self.data_recorder.close()

        if self.hdf5_recorder is not None:
            self.hdf5_recorder.close()

        if self.camera_renderer is not None:
            self.camera_renderer.close()
            self.camera_renderer = None
        
        cv2.destroyAllWindows()

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
