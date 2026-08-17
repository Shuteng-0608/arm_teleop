#!/usr/bin/env python3

from copy import deepcopy
import tempfile
import unittest
from pathlib import Path

from vptele.utils.mujoco_config import (
    MujocoConfigError,
    apply_runtime_overrides,
    build_arm_teleop_config,
    build_controller_config,
    load_mujoco_config,
    validate_mujoco_config,
)


REPO_ROOT = Path(__file__).resolve().parents[1]
CONFIG_PATH = REPO_ROOT / "vptele" / "config" / "config_arm_right_peg.yaml"
MOVING_HOLE_CONFIG_PATH = (
    REPO_ROOT / "vptele" / "config" / "config_arm_right_moving_hole.yaml"
)


class MujocoConfigTest(unittest.TestCase):
    def setUp(self):
        self.config = load_mujoco_config(str(CONFIG_PATH))

    def test_repository_config_is_valid_and_paths_are_absolute(self):
        validate_mujoco_config(self.config)
        self.assertEqual(
            Path(self.config["mujoco_model_path"]),
            REPO_ROOT / "model" / "pangu_all_right.xml",
        )
        record_dir = Path(self.config["hdf5_record_dir"])
        self.assertTrue(record_dir.is_absolute())
        self.assertEqual(record_dir.parent, REPO_ROOT / "data")

    def test_moving_hole_config_is_valid_and_role_mapped(self):
        config = load_mujoco_config(str(MOVING_HOLE_CONFIG_PATH))
        validate_mujoco_config(config)

        self.assertEqual(
            Path(config["mujoco_model_path"]),
            REPO_ROOT / "model" / "pangu_moving_hole_fixed_peg.xml",
        )
        self.assertEqual(config["hdf5_ee_body_name"], "hole_tool")
        self.assertEqual(config["hdf5_peg_geom_name"], "fixed_cylindrical_peg")
        self.assertEqual(config["task_moving_site_name"], "hole_goal_site")
        self.assertEqual(config["task_target_site_name"], "fixed_peg_tip_site")
        self.assertEqual(config["target_body_name"], "fixed_peg_fixture")
        self.assertEqual(config["task_success_distance"], 0.001)
        self.assertFalse(config["scripted_controller"]["enabled"])
        scripted = config["scripted_controller"]
        self.assertEqual(
            scripted["error_coverage_mode"], "stratified_radius_angle"
        )
        self.assertEqual(scripted["rim_contact_radii_mm"], [4.0, 6.0, 8.0, 10.0])
        self.assertEqual(scripted["rim_contact_angle_bins"], 24)
        self.assertLess(
            scripted["wall_contact_detect_force_n"],
            scripted["wall_contact_force_min"],
        )

    def test_operator_and_dataset_camera_roles_are_separated(self):
        self.assertEqual(self.config["visionpro_video_port"], 9999)
        self.assertIn("cctv_cam", self.config["monitor_camera_names"])
        self.assertEqual(
            self.config["hdf5_camera_names"],
            ["ee_cam", "base_top_cam"],
        )
        self.assertNotIn(
            "cctv_cam",
            self.config["hdf5_camera_names"],
        )

    def test_duplicate_yaml_keys_are_rejected(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            config_path = Path(temp_dir) / "duplicate.yaml"
            config_path.write_text("value: 1\nvalue: 2\n", encoding="utf-8")
            with self.assertRaisesRegex(MujocoConfigError, "Duplicate YAML key"):
                load_mujoco_config(str(config_path))

    def test_runtime_overrides_do_not_mutate_loaded_config(self):
        original = deepcopy(self.config)
        overridden = apply_runtime_overrides(
            self.config,
            vp_ip="10.0.0.8",
            end_effector="none",
            process_name="collector_a",
            review_mode="auto",
            target_episodes=12,
            max_attempts=40,
            reject_action="delete",
        )
        self.assertEqual(overridden["vp_ip"], "10.0.0.8")
        self.assertEqual(overridden["end_effector"], "none")
        self.assertEqual(overridden["logging"]["log_prefix"], "collector_a")
        scripted = overridden["scripted_controller"]
        self.assertEqual(scripted["review_mode"], "auto")
        self.assertEqual(scripted["target_episodes"], 12)
        self.assertEqual(scripted["max_attempts"], 40)
        self.assertEqual(scripted["reject_action"], "delete")
        self.assertEqual(self.config, original)

    def test_auto_mode_requires_positive_target(self):
        config = deepcopy(self.config)
        config["scripted_controller"]["review_mode"] = "auto"
        config["scripted_controller"]["target_episodes"] = 0
        with self.assertRaisesRegex(MujocoConfigError, "target_episodes > 0"):
            validate_mujoco_config(config)

    def test_all_lifecycle_and_hdf5_settings_reach_controller(self):
        config = deepcopy(self.config)
        config.update(
            {
                "teleop_controlled_by_recording": False,
                "accept_teleop_when_not_recording": True,
                "reset_arm_on_record_stop": False,
                "hdf5_record_actions": False,
                "hdf5_record_action_alias": False,
                "hdf5_enable_ft_tare": True,
                "hdf5_record_ft_wrench_raw": False,
                "hdf5_record_ft_wrench_gravity": False,
                "hdf5_ft_compensation_mode": "raw",
                "hdf5_ft_gravity_sensor_sign": 1.0,
                "hdf5_async_queue_size": 2048,
                "hdf5_async_stop_timeout": 5.0,
                "hdf5_append_block_image": 8,
                "render_queue_size": 3,
                "render_start_timeout": 5.0,
            }
        )
        config["arm_config"]["teleop_service_ns"] = "/custom_teleop"

        controller = build_controller_config(config)

        for key in (
            "teleop_controlled_by_recording",
            "accept_teleop_when_not_recording",
            "reset_arm_on_record_stop",
            "hdf5_record_actions",
            "hdf5_record_action_alias",
            "hdf5_enable_ft_tare",
            "hdf5_record_ft_wrench_raw",
            "hdf5_record_ft_wrench_gravity",
            "hdf5_ft_compensation_mode",
            "hdf5_ft_gravity_sensor_sign",
            "hdf5_async_queue_size",
            "hdf5_async_stop_timeout",
            "hdf5_append_block_image",
            "render_queue_size",
            "render_start_timeout",
        ):
            self.assertEqual(controller[key], config[key])

        self.assertEqual(
            controller["teleop_stop_service_name"], "/custom_teleop/stop"
        )
        self.assertEqual(
            controller["teleop_recalibrate_service_name"],
            "/custom_teleop/recalibrate",
        )
        self.assertEqual(
            controller["teleop_start_service_name"], "/custom_teleop/start"
        )
        self.assertTrue(controller["defer_runtime_activation"])

    def test_arm_config_uses_root_initial_joints(self):
        config = deepcopy(self.config)
        config["initial_arm_joints"] = [0.1] * 7
        arm_config = build_arm_teleop_config(config)
        self.assertEqual(arm_config["initial_arm_joints"], [0.1] * 7)

    def test_invalid_cross_field_config_is_rejected(self):
        config = deepcopy(self.config)
        config["record_hdf5"] = False
        config["enable_recording_service"] = True
        with self.assertRaisesRegex(MujocoConfigError, "requires record_hdf5"):
            validate_mujoco_config(config)

    def test_invalid_async_pipeline_capacity_is_rejected(self):
        config = deepcopy(self.config)
        config["render_queue_size"] = 0
        with self.assertRaisesRegex(
            MujocoConfigError, "render_queue_size must be positive"
        ):
            validate_mujoco_config(config)

    def test_invalid_visionpro_video_port_is_rejected_when_enabled(self):
        config = deepcopy(self.config)
        config["visionpro_video_enabled"] = True
        config["visionpro_video_port"] = 70000
        with self.assertRaisesRegex(
            MujocoConfigError, "visionpro_video_port must be between"
        ):
            validate_mujoco_config(config)

    def test_invalid_contact_coverage_is_rejected(self):
        config = deepcopy(self.config)
        scripted = config["scripted_controller"]
        scripted["error_coverage_mode"] = "stratified_radius_angle"
        scripted["rim_contact_radii_mm"] = [4.0, 4.0]
        scripted["rim_contact_angle_bins"] = 8
        with self.assertRaisesRegex(MujocoConfigError, "must not contain duplicates"):
            validate_mujoco_config(config)

        scripted["rim_contact_radii_mm"] = [4.0, 8.0]
        scripted["wall_contact_detect_force_n"] = 20.0
        scripted["wall_contact_force_min"] = 8.0
        scripted["wall_contact_force_max"] = 12.0
        with self.assertRaisesRegex(MujocoConfigError, "contact forces must satisfy"):
            validate_mujoco_config(config)

    def test_in_hole_correction_requires_gravity_compensation(self):
        config = load_mujoco_config(str(MOVING_HOLE_CONFIG_PATH))
        self.assertTrue(config["scripted_controller"]["in_hole_correction_enabled"])
        config["hdf5_ft_compensation_mode"] = "none"
        with self.assertRaisesRegex(MujocoConfigError, "requires.*gravity"):
            validate_mujoco_config(config)

    def test_invalid_in_hole_disturbance_range_is_rejected(self):
        config = load_mujoco_config(str(MOVING_HOLE_CONFIG_PATH))
        scripted = config["scripted_controller"]
        scripted["in_hole_disturbance_amplitudes_mm"] = [2.5]
        with self.assertRaisesRegex(MujocoConfigError, "exceed nominal clearance"):
            validate_mujoco_config(config)


if __name__ == "__main__":
    unittest.main()
