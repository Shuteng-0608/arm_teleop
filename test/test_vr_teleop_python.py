import argparse
import sys
import tempfile
import time
import unittest
from pathlib import Path

import mujoco
import numpy as np

from vptele.arm_control.arm_teleop_mujoco_python import (
    ArmTeleopMujocoPython,
    extract_wrist_transform,
)
from vptele.arm_control.mujoco_site_ik import MujocoSiteIK
from vptele.arm_control.robot_controller_mujoco_peg_tool_contact import (
    RobotControllerMuJoCoPegTool,
)
from vptele.core.synthetic_vp_streamer import SyntheticVPStreamer
from vptele.main_vr import build_vr_config


ROOT = Path(__file__).resolve().parents[1]
MODEL = ROOT / "model" / "pangu_moving_hole_fixed_peg.xml"
CONFIG = ROOT / "vptele" / "config" / "config_arm_right_moving_hole.yaml"
ARM_NAMES = [f"joint_{index}" for index in range(1, 8)]
ARM_SIGN = np.asarray([-1, 1, 1, -1, 1, 1, 1], dtype=float)
INITIAL_EXTERNAL = np.asarray(
    [-0.046, -0.2, 0.0, 1.6, -1.32, 0.005, 0.005], dtype=float
)


class _AttributeHand:
    def __init__(self, wrist):
        self.wrist = wrist


class _AttributeFrame:
    def __init__(self, wrist):
        self.right = _AttributeHand(wrist)


class _FixedStreamer:
    def __init__(self, frame):
        self.frame = frame

    def get_latest(self):
        return self.frame


class VisionProFrameTests(unittest.TestCase):
    def test_extracts_legacy_and_attribute_wrist_formats(self):
        wrist = np.eye(4)
        wrist[:3, 3] = [0.1, -0.2, 0.3]
        np.testing.assert_allclose(
            extract_wrist_transform({"right_wrist": wrist[np.newaxis]}), wrist
        )
        np.testing.assert_allclose(
            extract_wrist_transform(_AttributeFrame(wrist)), wrist
        )

    def test_synthetic_frame_matches_official_legacy_shapes(self):
        frame = SyntheticVPStreamer().get_latest()
        self.assertEqual(frame["head"].shape, (1, 4, 4))
        self.assertEqual(frame["right_wrist"].shape, (1, 4, 4))
        self.assertEqual(frame["right_fingers"].shape, (25, 4, 4))
        self.assertEqual(frame["right_arm"].shape, (27, 4, 4))


class MujocoIKTests(unittest.TestCase):
    def test_site_ik_reaches_small_cartesian_target(self):
        model = mujoco.MjModel.from_xml_path(str(MODEL))
        data = mujoco.MjData(model)
        joint_ids = [
            mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
            for name in ARM_NAMES
        ]
        qpos_addresses = model.jnt_qposadr[joint_ids]
        data.qpos[qpos_addresses] = INITIAL_EXTERNAL * ARM_SIGN
        mujoco.mj_forward(model, data)
        site_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_SITE, "hole_goal_site"
        )
        initial_position = data.site_xpos[site_id].copy()
        initial_rotation = data.site_xmat[site_id].reshape(3, 3).copy()
        target = initial_position + np.asarray([0.005, 0.0, 0.0])

        solver = MujocoSiteIK(
            model, ARM_NAMES, "hole_goal_site", gravity_compensation=False
        )
        solution = solver.solve(
            data.qpos[qpos_addresses],
            target,
            initial_rotation,
            base_qpos=data.qpos.copy(),
        )
        self.assertTrue(solution.converged)
        data.qpos[qpos_addresses] = solution.joints
        mujoco.mj_forward(model, data)
        self.assertLess(np.linalg.norm(data.site_xpos[site_id] - target), 1e-4)

    def test_teleop_maps_wrist_motion_without_ros(self):
        controller = RobotControllerMuJoCoPegTool(
            str(MODEL),
            {
                "auto_start": False,
                "launch_viewer": False,
                "record_hdf5": False,
                "record_data": False,
                "enable_ros_interfaces": False,
                "enable_recording_service": False,
                "defer_runtime_activation": True,
                "initial_arm_joints": INITIAL_EXTERNAL.tolist(),
                "arm_sign": ARM_SIGN.tolist(),
                "task_moving_site_name": "hole_goal_site",
                "enable_task_success_auto_stop": False,
                "reset_ignore_teleop_duration": 0.0,
                "show_camera_streams": False,
            },
        )
        wrist = np.eye(4)
        streamer = _FixedStreamer({"right_wrist": wrist[np.newaxis]})
        teleop = ArmTeleopMujocoPython(
            streamer,
            controller,
            {
                "ik_site_name": "hole_goal_site",
                "ik_gravity_compensation": False,
                "max_target_step": 0.02,
                "pose_filter_min_cutoff": 10000.0,
            },
        )
        self.assertTrue(teleop.calibrate())
        teleop.set_active(True)
        controller.accept_teleop_commands = True
        before = np.asarray(controller.target_joints[:7]).copy()
        moved = wrist.copy()
        moved[1, 3] = 0.00625  # legacy mapping -> +5 mm robot X
        solution = teleop.process_frame({"right_wrist": moved[np.newaxis]})
        self.assertIsNotNone(solution)
        self.assertTrue(solution.converged)
        self.assertGreater(np.linalg.norm(controller.target_joints[:7] - before), 1e-4)


class PurePythonEntryTests(unittest.TestCase):
    def test_entry_config_forces_ros_interfaces_off(self):
        options = argparse.Namespace(
            config=str(CONFIG), vp_ip=None, synthetic=True, headless=True
        )
        config = build_vr_config(options)
        self.assertFalse(config["enable_ros_interfaces"])
        self.assertFalse(config["enable_recording_service"])
        self.assertFalse(config["arm_config"]["enable_episode_services"])
        self.assertFalse(config["launch_viewer"])
        self.assertFalse(config["visionpro_video_enabled"])

    def test_importing_entry_does_not_load_rospy(self):
        self.assertNotIn("rospy", sys.modules)


if __name__ == "__main__":
    unittest.main()
