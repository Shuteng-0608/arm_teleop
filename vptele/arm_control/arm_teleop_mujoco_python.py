"""Pure-Python Vision Pro to MuJoCo arm teleoperation."""

from __future__ import annotations

import threading
import time
from typing import Any, Optional

import mujoco
import numpy as np

from vptele.arm_control.mujoco_site_ik import IKSolution, MujocoSiteIK
from vptele.utils.filters import OneEuroFilter
from vptele.utils.logger import get_logger


logger = get_logger()


def extract_wrist_transform(frame: Any, hand: str = "right") -> Optional[np.ndarray]:
    """Extract a finite 4x4 wrist transform from official old/new AVP frames."""
    if frame is None:
        return None
    key = f"{hand}_wrist"
    value = None
    getter = getattr(frame, "get", None)
    if callable(getter):
        value = getter(key, None)
    if value is None:
        hand_data = getattr(frame, hand, None)
        value = getattr(hand_data, "wrist", None) if hand_data is not None else None
    if value is None:
        return None
    array = np.asarray(value, dtype=float)
    if array.ndim == 3 and array.shape[0] > 0:
        array = array[0]
    if array.shape != (4, 4) or not np.all(np.isfinite(array)):
        return None
    return array.copy()


class ArmTeleopMujocoPython:
    """Map relative right-wrist motion to a MuJoCo-controlled task site.

    The coordinate mapping intentionally matches the legacy ROS teleoperator:
    AVP Y -> robot X, AVP Z -> robot Y, AVP X -> robot Z.  The tool orientation
    is calibrated from MuJoCo and held fixed, also matching the old behavior.
    """

    def __init__(self, vp_streamer, robot_controller, config=None) -> None:
        self.vp_streamer = vp_streamer
        self.robot_controller = robot_controller
        self.config = dict(config or {})
        self.update_period = float(self.config.get("update_frequency", 0.01))
        self.scaling_factor = float(self.config.get("scaling_factor", 0.8))
        self.translation_axes = np.asarray(
            self.config.get(
                "translation_axes",
                [[0.0, 1.0, 0.0], [0.0, 0.0, 0.8], [0.8, 0.0, 0.0]],
            ),
            dtype=float,
        ).reshape(3, 3)
        self.max_target_offset = np.asarray(
            self.config.get("max_target_offset", [0.20, 0.20, 0.20]),
            dtype=float,
        ).reshape(3)
        self.max_target_step = float(self.config.get("max_target_step", 0.01))
        self.pose_filter_min_cutoff = float(
            self.config.get("pose_filter_min_cutoff", 0.1)
        )
        self.pose_filter_beta = float(self.config.get("pose_filter_beta", 0.1))
        self.ik_failure_log_interval = float(
            self.config.get("ik_failure_log_interval", 1.0)
        )
        self.site_name = str(
            self.config.get(
                "ik_site_name", robot_controller.task_moving_site_name
            )
        )
        self.ik = MujocoSiteIK(
            robot_controller.model,
            robot_controller.arm_joint_names,
            self.site_name,
            damping=float(self.config.get("ik_damping", 1e-2)),
            max_iterations=int(self.config.get("ik_max_iterations", 80)),
            max_joint_step=float(self.config.get("ik_max_joint_step", 0.12)),
            position_tolerance=float(
                self.config.get("ik_position_tolerance", 5e-5)
            ),
            orientation_tolerance=float(
                self.config.get("ik_orientation_tolerance", 2e-3)
            ),
            orientation_weight=float(
                self.config.get("ik_orientation_weight", 0.35)
            ),
            gravity_compensation=bool(
                self.config.get("ik_gravity_compensation", True)
            ),
        )
        self._site_id = self.ik.site_id
        self._state_lock = threading.RLock()
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self.active = False
        self.calibrated = False
        self.initial_wrist_position = np.zeros(3)
        self.initial_site_position = np.zeros(3)
        self.initial_site_rotation = np.eye(3)
        self.last_target_position: Optional[np.ndarray] = None
        self.last_solution: Optional[IKSolution] = None
        self.last_frame_wall_time = 0.0
        self._last_failure_log = 0.0
        self._position_filter: Optional[OneEuroFilter] = None

    def _latest_frame(self):
        getter = getattr(self.vp_streamer, "get_latest", None)
        if callable(getter):
            return getter()
        return getattr(self.vp_streamer, "latest", None)

    def calibrate(self, frame: Any = None) -> bool:
        wrist = extract_wrist_transform(
            self._latest_frame() if frame is None else frame
        )
        if wrist is None:
            return False
        with self.robot_controller.lock:
            mujoco.mj_forward(
                self.robot_controller.model, self.robot_controller.data
            )
            site_position = self.robot_controller.data.site_xpos[
                self._site_id
            ].copy()
            site_rotation = self.robot_controller.data.site_xmat[
                self._site_id
            ].reshape(3, 3).copy()
        with self._state_lock:
            self.initial_wrist_position = wrist[:3, 3].copy()
            self.initial_site_position = site_position
            self.initial_site_rotation = site_rotation
            self.last_target_position = site_position.copy()
            self._position_filter = OneEuroFilter(
                time.time(),
                site_position,
                min_cutoff=self.pose_filter_min_cutoff,
                beta=self.pose_filter_beta,
            )
            self.calibrated = True
        logger.info(
            "Vision Pro teleop calibrated: wrist=%s site=%s",
            np.round(self.initial_wrist_position, 4).tolist(),
            np.round(self.initial_site_position, 4).tolist(),
        )
        return True

    def set_active(self, active: bool) -> None:
        with self._state_lock:
            self.active = bool(active)

    def process_frame(self, frame: Any) -> Optional[IKSolution]:
        wrist = extract_wrist_transform(frame)
        if wrist is None:
            return None
        with self._state_lock:
            if not self.active or not self.calibrated:
                return None
            offset = wrist[:3, 3] - self.initial_wrist_position
            target = (
                self.initial_site_position
                + self.scaling_factor * (self.translation_axes @ offset)
            )
            target = self.initial_site_position + np.clip(
                target - self.initial_site_position,
                -self.max_target_offset,
                self.max_target_offset,
            )
            if self._position_filter is not None:
                target = self._position_filter(time.time(), target)
            if self.last_target_position is not None:
                delta = target - self.last_target_position
                norm = float(np.linalg.norm(delta))
                if norm > self.max_target_step:
                    target = self.last_target_position + delta * (
                        self.max_target_step / norm
                    )
            target_rotation = self.initial_site_rotation.copy()

        with self.robot_controller.lock:
            base_qpos = self.robot_controller.data.qpos.copy()
            seed = self.robot_controller._get_current_arm_qpos_locked()
        solution = self.ik.solve(
            seed,
            target,
            target_rotation,
            base_qpos=base_qpos,
        )
        self.last_frame_wall_time = time.time()
        self.last_solution = solution
        if not solution.converged:
            if time.time() - self._last_failure_log >= self.ik_failure_log_interval:
                logger.warning(
                    "Vision Pro IK did not converge: position=%.6f m orientation=%.6f rad",
                    solution.position_error,
                    solution.orientation_error,
                )
                self._last_failure_log = time.time()
            return solution

        external = solution.joints * np.asarray(
            self.robot_controller.arm_sign, dtype=float
        )
        self.robot_controller.set_arm_positions(external.tolist())
        with self._state_lock:
            self.last_target_position = target.copy()
        return solution

    def _control_loop(self) -> None:
        deadline = time.perf_counter()
        while not self._stop_event.is_set():
            try:
                self.process_frame(self._latest_frame())
            except Exception:
                logger.exception("Vision Pro pure-Python teleop update failed")
            deadline += self.update_period
            wait = deadline - time.perf_counter()
            if wait <= 0:
                deadline = time.perf_counter()
            else:
                self._stop_event.wait(wait)

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop_event.clear()
        self._thread = threading.Thread(
            target=self._control_loop,
            name="visionpro-mujoco-teleop",
            daemon=True,
        )
        self._thread.start()

    def stop(self) -> None:
        self.set_active(False)
        self._stop_event.set()
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=2.0)
        self._thread = None
