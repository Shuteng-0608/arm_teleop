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
from vptele.main_mujoco_marvin import (
    build_python_only_config,
    select_marvin_ik_library_path,
)


REPO_ROOT = Path(__file__).resolve().parents[1]
CONFIG_PATH = REPO_ROOT / "vptele" / "config" / "config_arm_right_peg.yaml"
MARVIN_CONFIG_PATH = (
    REPO_ROOT / "vptele" / "config" / "config_arm_right_peg_marvin.yaml"
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
        self.assertEqual(
            Path(self.config["hdf5_record_dir"]),
            REPO_ROOT / "data" / "hole_random_60mm_hmj",
        )

    def test_operator_and_dataset_camera_roles_are_separated(self):
        self.assertFalse(self.config["launch_viewer"])
        self.assertTrue(self.config["visionpro_video_enabled"])
        self.assertEqual(self.config["visionpro_video_port"], 9999)
        self.assertEqual(self.config["monitor_camera_names"], ["cctv_cam"])
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

    def test_marvin_profile_inherits_base_and_overrides_robot_values(self):
        config = load_mujoco_config(str(MARVIN_CONFIG_PATH))
        validate_mujoco_config(config)

        self.assertEqual(
            Path(config["mujoco_model_path"]),
            REPO_ROOT / "model" / "pangu_all_right_marvin_m6.xml",
        )
        self.assertEqual(
            config["initial_arm_joints"],
            [-1.57, 1.57, 1.57, 1.6, -1.57, 0.0, 0.0],
        )
        self.assertEqual(config["arm_sign"], [1, -1, 1, -1, 1, 1, 1])
        self.assertEqual(config["arm_config"]["ik_backend"], "marvin_local")
        self.assertFalse(config["teleop_controlled_by_recording"])
        self.assertTrue(config["accept_teleop_when_not_recording"])
        self.assertFalse(config["enable_recording_service"])
        self.assertFalse(config["arm_config"]["enable_episode_services"])
        self.assertEqual(
            Path(config["arm_config"]["marvin_ik_module_path"]),
            REPO_ROOT / "ik_lib" / "fx_kine.py",
        )
        self.assertEqual(
            Path(config["arm_config"]["marvin_ik_library_path_windows"]),
            REPO_ROOT / "ik_lib" / "libKine.dll",
        )
        self.assertEqual(config["hdf5_camera_names"], ["ee_cam", "base_top_cam"])

    def test_marvin_library_selection_is_platform_specific(self):
        config = load_mujoco_config(str(MARVIN_CONFIG_PATH))
        arm_config = config["arm_config"]
        self.assertEqual(
            Path(select_marvin_ik_library_path(arm_config, "win32")),
            REPO_ROOT / "ik_lib" / "libKine.dll",
        )
        self.assertEqual(
            Path(select_marvin_ik_library_path(arm_config, "linux")),
            REPO_ROOT / "ik_lib" / "libKine.so",
        )

    def test_python_only_profile_disables_ros_and_can_auto_record(self):
        config = load_mujoco_config(str(MARVIN_CONFIG_PATH))
        standalone = build_python_only_config(
            config,
            vp_ip="10.0.0.42",
            record=True,
            label="python_test",
            viewer=True,
            camera_windows=False,
            video_return=False,
        )
        validate_mujoco_config(standalone)

        self.assertEqual(standalone["vp_ip"], "10.0.0.42")
        self.assertFalse(standalone["teleop_controlled_by_recording"])
        self.assertTrue(standalone["accept_teleop_when_not_recording"])
        self.assertFalse(standalone["enable_recording_service"])
        self.assertFalse(standalone["arm_config"]["enable_episode_services"])
        self.assertEqual(
            Path(standalone["arm_config"]["marvin_ik_library_path"]),
            REPO_ROOT / "ik_lib" / "libKine.dll",
        )
        self.assertTrue(standalone["hdf5_auto_start"])
        self.assertEqual(standalone["hdf5_episode_label"], "python_test")
        self.assertTrue(standalone["launch_viewer"])
        self.assertFalse(standalone["show_camera_streams"])
        self.assertFalse(standalone["visionpro_video_enabled"])

    def test_config_extends_cycle_is_rejected(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            first = Path(temp_dir) / "first.yaml"
            second = Path(temp_dir) / "second.yaml"
            first.write_text('extends: "second.yaml"\n', encoding="utf-8")
            second.write_text('extends: "first.yaml"\n', encoding="utf-8")
            with self.assertRaisesRegex(MujocoConfigError, "extends cycle"):
                load_mujoco_config(str(first))

    def test_inherited_relative_paths_stay_relative_to_base_file(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            profile_path = Path(temp_dir) / "profile.yaml"
            profile_path.write_text(
                f'extends: "{CONFIG_PATH.as_posix()}"\nvp_ip: "10.0.0.9"\n',
                encoding="utf-8",
            )
            config = load_mujoco_config(str(profile_path))
            self.assertEqual(
                Path(config["mujoco_model_path"]),
                REPO_ROOT / "model" / "pangu_all_right.xml",
            )

    def test_runtime_overrides_do_not_mutate_loaded_config(self):
        original = deepcopy(self.config)
        overridden = apply_runtime_overrides(
            self.config,
            vp_ip="10.0.0.8",
            end_effector="none",
            process_name="collector_a",
        )
        self.assertEqual(overridden["vp_ip"], "10.0.0.8")
        self.assertEqual(overridden["end_effector"], "none")
        self.assertEqual(overridden["logging"]["log_prefix"], "collector_a")
        self.assertEqual(self.config, original)

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


if __name__ == "__main__":
    unittest.main()
