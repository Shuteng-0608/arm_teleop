"""ROS-free MuJoCo system used by automatic scripted data collection."""

from __future__ import annotations

import threading
from copy import deepcopy
from typing import Any, Dict, Optional

from vptele.arm_control.robot_controller_mujoco_peg_tool_contact import (
    RobotControllerMuJoCoPegTool,
)
from vptele.arm_control.scripted_collection import ScriptedInsertionRunner
from vptele.utils.logger import get_logger
from vptele.utils.mujoco_config import build_controller_config


logger = get_logger()


class MujocoAutomaticCollectionSystem:
    """Own the simulation, recorder, and one automatic batch lifecycle."""

    def __init__(
        self,
        config: Dict[str, Any],
        *,
        stop_event: Optional[threading.Event] = None,
    ) -> None:
        self.config = deepcopy(config)
        self.stop_event = stop_event or threading.Event()
        self.robot_controller = None
        self.runner = None

    def initialize(self) -> None:
        controller_config = build_controller_config(self.config)
        controller_config.update(
            {
                "enable_ros_interfaces": False,
                "enable_recording_service": False,
                "defer_runtime_activation": True,
                "vp_enabled": False,
                "visionpro_video_enabled": False,
            }
        )

        model_path = self.config["mujoco_model_path"]
        logger.info("Initializing standalone MuJoCo model: %s", model_path)
        self.robot_controller = RobotControllerMuJoCoPegTool(
            model_path=model_path,
            config=controller_config,
        )

        scripted_config = dict(self.config.get("scripted_controller") or {})
        scripted_config.setdefault("config_path", self.config.get("config_path", ""))
        scripted_config.setdefault(
            "hdf5_record_dir", self.config.get("hdf5_record_dir", "")
        )
        scripted_config.setdefault(
            "initial_robot_pose",
            self.config.get("arm_config", {}).get("initial_robot_pose"),
        )
        scripted_config.setdefault(
            "reset_ignore_teleop_duration",
            self.config.get("reset_ignore_teleop_duration", 0.5),
        )
        self.runner = ScriptedInsertionRunner(
            robot_controller=self.robot_controller,
            config=scripted_config,
            stop_event=self.stop_event,
        )
        self.robot_controller.activate_runtime()

    def run(self) -> bool:
        if self.runner is None:
            raise RuntimeError("Collection system must be initialized before run()")
        if self.stop_event.is_set():
            return False
        if not self.runner.start_automatic_batch():
            raise RuntimeError("Automatic batch did not start")

        while not self.runner.batch_finished_event.wait(timeout=0.25):
            if self.stop_event.is_set():
                self.runner.stop_automatic_batch()
        return bool(self.runner.batch_completed)

    @property
    def batch_stats(self) -> Dict[str, int]:
        if self.runner is None:
            return {"attempted": 0, "kept": 0, "rejected": 0}
        return dict(self.runner.batch_stats)

    @property
    def manifest_path(self) -> str:
        if self.runner is None or self.runner._batch_manifest_path is None:
            return ""
        return str(self.runner._batch_manifest_path)

    def stop(self) -> None:
        self.stop_event.set()
        if self.runner is not None:
            self.runner.stop_automatic_batch()
        if self.robot_controller is not None:
            self.robot_controller.disconnect()
