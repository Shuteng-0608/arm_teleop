#!/usr/bin/env python3

import unittest
from pathlib import Path

import mujoco
import numpy as np

from vptele.arm_control.robot_controller_mujoco_peg_tool_contact import (
    RobotControllerMuJoCoPegTool,
)
from vptele.utils.ft_wrench_utils import raw_ft_wrench
from vptele.utils.mujoco_config import build_controller_config, load_mujoco_config


REPO_ROOT = Path(__file__).resolve().parents[1]
CONFIG_PATH = REPO_ROOT / "vptele" / "config" / "config_arm_right_moving_hole.yaml"


class MovingHoleHudTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        config = build_controller_config(load_mujoco_config(str(CONFIG_PATH)))
        config.update(
            {
                "auto_start": False,
                "launch_viewer": False,
                "show_camera_streams": False,
                "record_hdf5": False,
                "enable_ros_interfaces": False,
                "enable_recording_service": False,
            }
        )
        cls.controller = RobotControllerMuJoCoPegTool(
            config["mujoco_model_path"],
            config,
        )
        mujoco.mj_forward(cls.controller.model, cls.controller.data)

    @classmethod
    def tearDownClass(cls):
        cls.controller.disconnect()

    def test_configured_hole_ft_sensors_feed_hud(self):
        controller = self.controller
        wrench = raw_ft_wrench(
            controller.model,
            controller.data,
            force_sensor_name=controller.ft_force_sensor_name,
            torque_sensor_name=controller.ft_torque_sensor_name,
        )

        self.assertIsNotNone(wrench)
        self.assertEqual(wrench.shape, (6,))
        feedback = controller._get_force_feedback_snapshot(controller.data)
        self.assertIsNotNone(feedback)
        self.assertEqual(feedback["source_label"], "comp")
        self.assertTrue(np.isfinite(feedback["force_norm"]))
        self.assertEqual(
            controller.force_feedback_config.hud_title,
            "Hole Contact HUD",
        )

    def test_cctv_frame_contains_force_feedback_overlay(self):
        controller = self.controller
        feedback = controller._get_force_feedback_snapshot(controller.data)
        frame_rgb = np.zeros((480, 640, 3), dtype=np.uint8)

        with_hud = controller._render_cctv_window_bgr(
            camera_name="cctv_cam",
            feedback=feedback,
            frames_rgb={"cctv_cam": frame_rgb},
        )
        controller.force_feedback_config.enabled = False
        try:
            without_hud = controller._render_cctv_window_bgr(
                camera_name="cctv_cam",
                feedback=feedback,
                frames_rgb={"cctv_cam": frame_rgb},
            )
        finally:
            controller.force_feedback_config.enabled = True

        self.assertEqual(with_hud.shape, (720, 1280, 3))
        changed_pixels = np.count_nonzero(
            np.any(with_hud != without_hud, axis=2)
        )
        self.assertGreater(changed_pixels, 1000)


if __name__ == "__main__":
    unittest.main()
