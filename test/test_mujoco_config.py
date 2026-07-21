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


if __name__ == "__main__":
    unittest.main()
