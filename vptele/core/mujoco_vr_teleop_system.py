"""ROS-free Vision Pro teleoperation and HDF5 episode lifecycle."""

from __future__ import annotations

import shutil
import threading
import time
from copy import deepcopy
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Optional

import numpy as np

from vptele.arm_control.arm_teleop_mujoco_python import (
    ArmTeleopMujocoPython,
    extract_wrist_transform,
)
from vptele.arm_control.robot_controller_mujoco_peg_tool_contact import (
    RobotControllerMuJoCoPegTool,
)
from vptele.utils.logger import get_logger
from vptele.utils.mujoco_config import (
    build_arm_teleop_config,
    build_controller_config,
)


logger = get_logger()


@dataclass(frozen=True)
class EpisodeOutcome:
    success: bool
    active: bool
    message: str
    episode_path: str = ""


class MujocoVRTeleopSystem:
    """Own Vision Pro input, Python IK, simulation, and manual recording."""

    def __init__(self, config: Dict[str, Any], *, vp_streamer=None) -> None:
        self.config = deepcopy(config)
        self.vp_streamer = vp_streamer
        self.robot_controller: Optional[RobotControllerMuJoCoPegTool] = None
        self.arm_teleop: Optional[ArmTeleopMujocoPython] = None
        self._owns_streamer = vp_streamer is None

    def initialize(self) -> None:
        if self.vp_streamer is None:
            from vptele.core.vp_streamer_avp import VPStreamer

            self.vp_streamer = VPStreamer(
                ip=self.config.get("vp_ip"),
                record=bool(self.config.get("vp_record", False)),
            )
        self._wait_for_tracking(float(self.config.get("vp_ready_timeout", 5.0)))

        controller_config = build_controller_config(self.config)
        controller_config.update(
            {
                "enable_ros_interfaces": False,
                "enable_recording_service": False,
                "defer_runtime_activation": True,
            }
        )
        self.robot_controller = RobotControllerMuJoCoPegTool(
            model_path=self.config["mujoco_model_path"],
            config=controller_config,
        )
        arm_config = build_arm_teleop_config(self.config)
        arm_config.setdefault(
            "ik_site_name", self.config.get("task_moving_site_name", "peg_tip_site")
        )
        self.arm_teleop = ArmTeleopMujocoPython(
            self.vp_streamer, self.robot_controller, arm_config
        )
        if not self.arm_teleop.calibrate():
            raise TimeoutError("Vision Pro right-wrist frame disappeared during calibration")

        self._initialize_video_return()
        self.robot_controller.activate_runtime()
        self.arm_teleop.start()
        self.arm_teleop.set_active(
            bool(self.config.get("accept_teleop_when_not_recording", False))
        )
        logger.info("ROS-free Vision Pro MuJoCo teleoperation is ready")

    def _wait_for_tracking(self, timeout: float) -> None:
        deadline = time.monotonic() + max(timeout, 0.01)
        while time.monotonic() < deadline:
            getter = getattr(self.vp_streamer, "get_latest", None)
            frame = getter() if callable(getter) else getattr(
                self.vp_streamer, "latest", None
            )
            if extract_wrist_transform(frame) is not None:
                return
            time.sleep(0.02)
        raise TimeoutError("Vision Pro tracking timeout: no valid right_wrist pose")

    def _initialize_video_return(self) -> None:
        if not bool(self.config.get("visionpro_video_enabled", False)):
            return
        start_video = getattr(self.vp_streamer, "start_video_stream", None)
        update_video = getattr(self.vp_streamer, "update_video_frame", None)
        if not callable(start_video) or not callable(update_video):
            logger.warning("Tracking source does not support CCTV video return")
            return
        if not self.robot_controller.set_cctv_frame_sink(update_video):
            logger.warning("Configured CCTV camera was not found; video return disabled")
            return
        started = start_video(
            width=self.config.get("cctv_window_width", 1280),
            height=self.config.get("cctv_window_height", 720),
            fps=self.config.get("camera_stream_fps", 15.0),
            port=self.config.get("visionpro_video_port", 9999),
        )
        if not started:
            self.robot_controller.set_cctv_frame_sink(None)
            logger.warning("Vision Pro video return unavailable; tracking remains active")

    @property
    def recording_active(self) -> bool:
        recorder = getattr(self.robot_controller, "hdf5_recorder", None)
        return bool(recorder is not None and recorder.active)

    @property
    def pending_review(self) -> bool:
        return bool(
            self.robot_controller is not None
            and self.robot_controller.pending_auto_completed_review
        )

    def start_episode(self, label: str = "teleop") -> EpisodeOutcome:
        rc = self.robot_controller
        teleop = self.arm_teleop
        if rc is None or teleop is None:
            return EpisodeOutcome(False, False, "System is not initialized")
        recorder = rc.hdf5_recorder
        if recorder is None:
            return EpisodeOutcome(False, False, "HDF5 recorder is not initialized")
        if recorder.active:
            path = str(recorder.hdf5_path or "")
            return EpisodeOutcome(False, True, "Recording is already active", path)
        if rc.pending_auto_completed_review:
            return EpisodeOutcome(
                False,
                False,
                "Review the auto-completed episode before starting another",
                rc.pending_auto_completed_episode_path,
            )

        teleop.set_active(False)
        rc.accept_teleop_commands = False
        if rc.reset_arm_on_record_start:
            rc.reset_arm_to_initial_pose()
        if rc.reset_joint_torque_alarm_on_record_start:
            rc.reset_joint_torque_alarm()
        if rc.reset_ft_wrench_alarm_on_record_start:
            rc.reset_ft_wrench_alarm()
        with rc.lock:
            rc._reset_task_success_state_locked()

        if not teleop.calibrate():
            return EpisodeOutcome(False, False, "No valid right-wrist frame for calibration")
        with rc.lock:
            path = recorder.start_episode(
                label=(label.strip() or "teleop"),
                episode_metadata=rc.current_hole_sample,
            )
        rc.accept_teleop_commands = True
        teleop.set_active(True)
        return EpisodeOutcome(
            True,
            True,
            f"Started recording: {label.strip() or 'teleop'}",
            str(path or ""),
        )

    def stop_episode(self, *, keep: bool) -> EpisodeOutcome:
        rc = self.robot_controller
        teleop = self.arm_teleop
        if rc is None or teleop is None or rc.hdf5_recorder is None:
            return EpisodeOutcome(False, False, "System is not initialized")
        recorder = rc.hdf5_recorder
        rc.accept_teleop_commands = False
        teleop.set_active(False)

        if recorder.active:
            path_before = recorder.hdf5_path
            with rc.lock:
                path = recorder.stop_episode(
                    status="manual_keep" if keep else "manual_discard"
                )
            path = path or path_before
        elif rc.pending_auto_completed_review:
            path = (
                Path(rc.pending_auto_completed_episode_path)
                if rc.pending_auto_completed_episode_path
                else None
            )
        else:
            return EpisodeOutcome(False, False, "No episode is awaiting review")

        if path is not None and not isinstance(path, Path):
            path = Path(path)
        episode_path = str(path or "")
        episode_dir = path.parent if path is not None else None
        rc._reset_after_episode_stop()
        next_target = rc.finalize_hole_after_episode(keep=keep)
        if not keep and episode_dir is not None and episode_dir.exists():
            shutil.rmtree(episode_dir, ignore_errors=True)

        rc.pending_auto_completed_review = False
        rc.pending_auto_completed_episode_path = ""
        rc.pending_auto_completed_episode_dir = ""
        cell = next_target.get("hole_grid_cell_label", "n/a")
        action = "kept" if keep else "discarded"
        return EpisodeOutcome(
            True,
            False,
            f"Episode {action}; next target: {cell}",
            episode_path,
        )

    def recalibrate(self) -> bool:
        if self.arm_teleop is None:
            return False
        was_active = self.arm_teleop.active
        self.arm_teleop.set_active(False)
        calibrated = self.arm_teleop.calibrate()
        self.arm_teleop.set_active(was_active and calibrated)
        return calibrated

    def stop(self) -> None:
        if self.arm_teleop is not None:
            self.arm_teleop.stop()
        if self.robot_controller is not None:
            self.robot_controller.accept_teleop_commands = False
            if self.recording_active:
                with self.robot_controller.lock:
                    self.robot_controller.hdf5_recorder.stop_episode(
                        status="interrupted"
                    )
            self.robot_controller.disconnect()
        if self.vp_streamer is not None:
            try:
                self.vp_streamer.close()
            except Exception:
                logger.exception("Failed to close Vision Pro streamer")
