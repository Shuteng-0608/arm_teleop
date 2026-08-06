#!/usr/bin/env python3
"""Vision Pro teleoperation for the Marvin M6 MuJoCo model and local IK."""

from __future__ import annotations

import threading
import time
from typing import Optional

import numpy as np
try:
    import rospy
    from std_srvs.srv import Trigger, TriggerResponse

    ROS_AVAILABLE = True
except ImportError:
    rospy = None
    Trigger = None
    ROS_AVAILABLE = False

    class TriggerResponse:
        def __init__(self, success=False, message=""):
            self.success = bool(success)
            self.message = str(message)

from arm_control.marvin_ik_solver import (
    MarvinIKNoSolution,
    MarvinIKSolver,
)
from utils.filters import OneEuroFilter
from utils.logger import get_logger


LOGGER = get_logger()


def _first_hand_transform(value) -> Optional[np.ndarray]:
    """Normalize AVP's possible 4x4 or Nx4x4 wrist representation."""
    if value is None:
        return None
    matrix = np.asarray(value, dtype=np.float64)
    if matrix.shape == (4, 4):
        return matrix.copy()
    if matrix.ndim == 3 and matrix.shape[0] > 0 and matrix.shape[1:] == (4, 4):
        return matrix[0].copy()
    return None


def vendor_joints_to_controller_command(vendor_joints, arm_sign) -> np.ndarray:
    """Convert SDK/MuJoCo joint coordinates to controller command coordinates."""
    joints = np.asarray(vendor_joints, dtype=np.float64)
    signs = np.asarray(arm_sign, dtype=np.float64)
    if joints.shape != (7,) or not np.all(np.isfinite(joints)):
        raise ValueError("vendor_joints must contain seven finite values")
    if signs.shape != (7,) or not np.all(np.isfinite(signs)):
        raise ValueError("arm_sign must contain seven finite values")
    if np.any(np.abs(signs) < 1e-12):
        raise ValueError("arm_sign cannot contain zero")
    # Controller later computes internal = command * arm_sign.
    return joints / signs


class ArmTeleopMujocoMarvin:
    """Map right-wrist translation to Marvin SDK IK and MuJoCo actuators.

    The target orientation remains the SDK FK orientation at episode start.
    Only translation is teleoperated, matching the established peg-collection
    behavior. SDK FK is also used to derive the SDK-base/world vector mapping
    from the MuJoCo flange pose instead of hard-coding a mount rotation.
    """

    def __init__(self, vp_streamer, robot_controller, config=None):
        self.vp_streamer = vp_streamer
        self.robot_controller = robot_controller
        self.config = dict(config or {})

        self.update_frequency = float(self.config.get("update_frequency", 0.01))
        self.scaling_factor = float(self.config.get("scaling_factor", 0.8))
        self.axis_scale = np.asarray(
            self.config.get("marvin_hand_axis_scale", [1.0, 0.8, 0.8]),
            dtype=np.float64,
        )
        if self.axis_scale.shape != (3,) or not np.all(np.isfinite(self.axis_scale)):
            raise ValueError("marvin_hand_axis_scale must contain three finite values")

        self.translation_units_per_meter = float(
            self.config.get("marvin_translation_units_per_meter", 1000.0)
        )
        self.max_ik_joint_jump_rad = float(
            self.config.get("marvin_max_ik_joint_jump_rad", 0.75)
        )
        self.flange_body_name = str(
            self.config.get("marvin_mujoco_flange_body_name", "link_7")
        )
        self.controller_arm_sign = np.asarray(
            getattr(
                self.robot_controller,
                "arm_sign",
                [1.0, -1.0, 1.0, -1.0, 1.0, 1.0, 1.0],
            ),
            dtype=np.float64,
        )
        # Validate the conversion before loading/using the vendor solver.
        vendor_joints_to_controller_command(
            np.zeros(7, dtype=np.float64),
            self.controller_arm_sign,
        )

        tool_matrix = self.config.get("marvin_tool_matrix")
        self.ik_solver = MarvinIKSolver(
            module_path=self.config["marvin_ik_module_path"],
            library_path=self.config["marvin_ik_library_path"],
            config_path=self.config["marvin_ik_config_path"],
            arm_type=int(self.config.get("marvin_arm_type", 1)),
            zsp_type=int(self.config.get("marvin_zsp_type", 0)),
            zsp_para=self.config.get("marvin_zsp_para", [0.0] * 6),
            zsp_angle_deg=float(
                self.config.get("marvin_zsp_angle_deg", 0.0)
            ),
            reject_singular=bool(
                self.config.get("marvin_reject_singular", True)
            ),
            reject_joint_limit=bool(
                self.config.get("marvin_reject_joint_limit", True)
            ),
            tool_matrix=tool_matrix,
        )

        initial_command_joints = np.asarray(
            self.config.get(
                "initial_arm_joints",
                [-1.57, 1.57, 1.57, 1.6, -1.57, 0.0, 0.0],
            ),
            dtype=np.float64,
        )
        if initial_command_joints.shape != (7,) or not np.all(
            np.isfinite(initial_command_joints)
        ):
            raise ValueError("initial_arm_joints must contain seven values")
        self.initial_joints = initial_command_joints * self.controller_arm_sign
        self.last_joint_solution = self.initial_joints.copy()

        self.filter_min_cutoff = float(
            self.config.get("pose_filter_min_cutoff", 0.1)
        )
        self.filter_beta = float(self.config.get("pose_filter_beta", 0.1))
        self.translation_filter = None

        self.running = False
        self.teleop_active = True
        self.control_thread: Optional[threading.Thread] = None
        self.initial_hand_position = np.zeros(3, dtype=np.float64)
        self.initial_ik_pose = np.eye(4, dtype=np.float64)
        self.world_to_ik_rotation = np.eye(3, dtype=np.float64)
        self.last_warning_wall_time = 0.0

        self.recalibrate_for_new_episode()
        self._register_episode_services()
        LOGGER.info("Marvin local-IK teleoperation initialized")

    def _register_episode_services(self) -> None:
        self.teleop_service_ns = str(
            self.config.get("teleop_service_ns", "/arm_teleop_mujoco")
        ).rstrip("/")
        self.enable_episode_services = bool(
            self.config.get("enable_episode_services", True)
        )
        if not self.enable_episode_services:
            return
        if not ROS_AVAILABLE:
            raise RuntimeError(
                "enable_episode_services=true requires ROS. Set it to false "
                "when using the Python-only Marvin entry."
            )

        self.srv_stop_teleop = rospy.Service(
            f"{self.teleop_service_ns}/stop",
            Trigger,
            self._handle_stop_teleop_service,
        )
        self.srv_recalibrate_teleop = rospy.Service(
            f"{self.teleop_service_ns}/recalibrate",
            Trigger,
            self._handle_recalibrate_teleop_service,
        )
        self.srv_start_teleop = rospy.Service(
            f"{self.teleop_service_ns}/start",
            Trigger,
            self._handle_start_teleop_service,
        )

    def _calibrate_hand_position(self) -> None:
        for attempt in range(10):
            transform = _first_hand_transform(
                self.vp_streamer.get_hand_position(hand="right")
            )
            if transform is not None:
                self.initial_hand_position = transform[:3, 3].copy()
                LOGGER.info(
                    "Marvin teleop hand reference calibrated: %s",
                    self.initial_hand_position.tolist(),
                )
                return
            LOGGER.info("Waiting for right wrist data: %d/10", attempt + 1)
            time.sleep(0.5)
        raise RuntimeError("Unable to calibrate Vision Pro right wrist")

    def _update_kinematic_reference(self) -> None:
        self.initial_ik_pose = self.ik_solver.forward(self.initial_joints)
        mujoco_flange_pose = self.robot_controller.get_body_pose_matrix(
            self.flange_body_name
        )
        if mujoco_flange_pose is None:
            raise ValueError(
                f"MuJoCo flange body not found: {self.flange_body_name}"
            )

        # v_sdk = R_sdk_flange * R_world_flange^T * v_world
        self.world_to_ik_rotation = (
            self.initial_ik_pose[:3, :3]
            @ mujoco_flange_pose[:3, :3].T
        )
        if not np.all(np.isfinite(self.world_to_ik_rotation)):
            raise RuntimeError("Invalid MuJoCo-to-Marvin frame calibration")

    def recalibrate_for_new_episode(self):
        self.teleop_active = False
        if hasattr(self.robot_controller, "initial_arm_joints_internal"):
            self.initial_joints = np.asarray(
                self.robot_controller.initial_arm_joints_internal,
                dtype=np.float64,
            )
        self.last_joint_solution = self.initial_joints.copy()
        self._update_kinematic_reference()
        self._calibrate_hand_position()
        self.translation_filter = OneEuroFilter(
            time.time(),
            np.zeros(3, dtype=np.float64),
            min_cutoff=self.filter_min_cutoff,
            beta=self.filter_beta,
        )
        self.teleop_active = True

    def _hand_offset_world(self, hand_transform: np.ndarray) -> np.ndarray:
        hand_offset = hand_transform[:3, 3] - self.initial_hand_position
        # Preserve the established AVP mapping in MuJoCo world coordinates.
        return np.asarray(
            [
                hand_offset[1] * self.scaling_factor * self.axis_scale[0],
                hand_offset[2] * self.scaling_factor * self.axis_scale[1],
                hand_offset[0] * self.scaling_factor * self.axis_scale[2],
            ],
            dtype=np.float64,
        )

    def _target_pose(self, hand_transform: np.ndarray, timestamp: float) -> np.ndarray:
        world_offset = self._hand_offset_world(hand_transform)
        filtered_world_offset = self.translation_filter(timestamp, world_offset)
        ik_offset = (
            self.world_to_ik_rotation
            @ filtered_world_offset
            * self.translation_units_per_meter
        )
        target = self.initial_ik_pose.copy()
        target[:3, 3] += ik_offset
        return target

    def _log_warning_throttled(self, message: str) -> None:
        now = time.time()
        if now - self.last_warning_wall_time >= 1.0:
            LOGGER.warning(message)
            self.last_warning_wall_time = now

    def control_loop(self):
        LOGGER.info("Starting Marvin Vision Pro control loop")
        while self.running:
            loop_started = time.perf_counter()
            try:
                hand_transform = _first_hand_transform(
                    self.vp_streamer.get_hand_position(hand="right")
                )
                if self.teleop_active and hand_transform is not None:
                    target = self._target_pose(hand_transform, time.time())
                    solution = self.ik_solver.solve(
                        target,
                        self.last_joint_solution,
                    )
                    jump = float(
                        np.max(np.abs(solution - self.last_joint_solution))
                    )
                    if jump > self.max_ik_joint_jump_rad:
                        self._log_warning_throttled(
                            "Rejected Marvin IK branch jump: "
                            f"{jump:.4f} rad > {self.max_ik_joint_jump_rad:.4f} rad"
                        )
                    else:
                        self.last_joint_solution = solution.copy()
                        command = vendor_joints_to_controller_command(
                            solution,
                            self.controller_arm_sign,
                        )
                        self.robot_controller.set_arm_positions(command.tolist())
            except MarvinIKNoSolution as exc:
                self._log_warning_throttled(f"Marvin IK rejected target: {exc}")
            except Exception as exc:
                LOGGER.exception("Marvin teleoperation loop failed: %s", exc)
                time.sleep(0.2)

            remaining = self.update_frequency - (
                time.perf_counter() - loop_started
            )
            if remaining > 0.0:
                time.sleep(remaining)

    def start(self):
        if self.running:
            return
        self.running = True
        self.control_thread = threading.Thread(
            target=self.control_loop,
            name="MarvinArmTeleopThread",
            daemon=True,
        )
        self.control_thread.start()

    def stop(self):
        self.teleop_active = False
        self.running = False
        if self.control_thread is not None:
            self.control_thread.join(timeout=2.0)
        self.control_thread = None

    def _handle_stop_teleop_service(self, _request):
        try:
            self.stop()
            return TriggerResponse(True, "Marvin arm teleoperation stopped.")
        except Exception as exc:
            return TriggerResponse(False, f"Failed to stop Marvin teleoperation: {exc}")

    def _handle_recalibrate_teleop_service(self, _request):
        try:
            if self.running:
                self.stop()
            self.recalibrate_for_new_episode()
            return TriggerResponse(True, "Marvin teleoperation recalibrated.")
        except Exception as exc:
            LOGGER.exception("Marvin recalibration failed: %s", exc)
            return TriggerResponse(False, f"Failed to recalibrate Marvin teleoperation: {exc}")

    def _handle_start_teleop_service(self, _request):
        try:
            self.teleop_active = True
            self.start()
            return TriggerResponse(True, "Marvin arm teleoperation started.")
        except Exception as exc:
            return TriggerResponse(False, f"Failed to start Marvin teleoperation: {exc}")
