#!/usr/bin/env python3

import tempfile
import unittest
from pathlib import Path

import h5py
import mujoco
import numpy as np

from vptele.utils.mujoco_hdf5_recorder import MujocoHDF5Recorder
from vptele.utils.mujoco_config import build_controller_config, load_mujoco_config


REPO_ROOT = Path(__file__).resolve().parents[1]
MODEL_PATH = REPO_ROOT / "model" / "pangu_moving_hole_fixed_peg.xml"
CONFIG_PATH = (
    REPO_ROOT / "vptele" / "config" / "config_arm_right_moving_hole.yaml"
)


class MovingHoleModelTest(unittest.TestCase):
    def setUp(self):
        self.model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
        self.data = mujoco.MjData(self.model)
        mujoco.mj_forward(self.model, self.data)

    def _id(self, object_type, name):
        object_id = mujoco.mj_name2id(self.model, object_type, name)
        self.assertNotEqual(object_id, -1, name)
        return object_id

    def test_task_roles_and_geometry_are_wired_correctly(self):
        hole_body = self._id(mujoco.mjtObj.mjOBJ_BODY, "hole_tool")
        link7_body = self._id(mujoco.mjtObj.mjOBJ_BODY, "link_7")
        fixture_body = self._id(mujoco.mjtObj.mjOBJ_BODY, "fixed_peg_fixture")
        peg_geom = self._id(mujoco.mjtObj.mjOBJ_GEOM, "fixed_cylindrical_peg")

        self.assertEqual(int(self.model.body_parentid[hole_body]), link7_body)
        self.assertEqual(int(self.model.body_parentid[fixture_body]), 0)
        self.assertEqual(
            int(self.model.geom_type[peg_geom]),
            int(mujoco.mjtGeom.mjGEOM_CYLINDER),
        )
        np.testing.assert_allclose(
            self.model.geom_size[peg_geom, :2], [0.011, 0.0225]
        )
        self.assertAlmostEqual(float(self.model.body_mass[hole_body]), 0.085)

        ring_ids = []
        for geom_id in range(self.model.ngeom):
            name = mujoco.mj_id2name(
                self.model, mujoco.mjtObj.mjOBJ_GEOM, geom_id
            )
            if name and name.startswith("tool_hole_ring_"):
                ring_ids.append(geom_id)
        self.assertEqual(len(ring_ids), 24)
        self.assertTrue(
            all(
                int(self.model.geom_bodyid[geom_id]) == hole_body
                for geom_id in ring_ids
            )
        )

    def test_visual_mount_bridges_link_and_socket_without_collision(self):
        def geom_interval(name):
            geom_id = self._id(mujoco.mjtObj.mjOBJ_GEOM, name)
            self.assertEqual(
                int(self.model.geom_type[geom_id]),
                int(mujoco.mjtGeom.mjGEOM_CYLINDER),
            )
            self.assertEqual(int(self.model.geom_contype[geom_id]), 0)
            self.assertEqual(int(self.model.geom_conaffinity[geom_id]), 0)
            center_z = float(self.model.geom_pos[geom_id, 2])
            half_length = float(self.model.geom_size[geom_id, 1])
            return center_z - half_length, center_z + half_length

        stem_min, stem_max = geom_interval("hole_tool_mount_stem_visual")
        flange_min, flange_max = geom_interval("hole_tool_mount_flange_visual")

        back_stop_id = self._id(
            mujoco.mjtObj.mjOBJ_GEOM,
            "tool_hole_back_stop",
        )
        back_stop_rear = float(
            self.model.geom_pos[back_stop_id, 2]
            + self.model.geom_size[back_stop_id, 1]
        )

        self.assertAlmostEqual(stem_max, 0.0, places=9)
        self.assertAlmostEqual(stem_min, flange_max, places=9)
        self.assertAlmostEqual(flange_min, back_stop_rear, places=9)

    def test_approach_and_success_sites_leave_seating_clearance(self):
        tip = self._id(mujoco.mjtObj.mjOBJ_SITE, "fixed_peg_tip_site")
        approach = self._id(
            mujoco.mjtObj.mjOBJ_SITE, "fixed_peg_approach_goal_site"
        )
        goal = self._id(mujoco.mjtObj.mjOBJ_SITE, "hole_goal_site")
        entrance = self._id(mujoco.mjtObj.mjOBJ_SITE, "hole_entrance_site")

        np.testing.assert_allclose(
            self.data.site_xpos[approach] - self.data.site_xpos[tip],
            [0.0, 0.043, 0.0],
            atol=1e-12,
        )
        self.assertAlmostEqual(
            float(
                np.linalg.norm(
                    self.model.site_pos[entrance] - self.model.site_pos[goal]
                )
            ),
            0.043,
        )

    def test_dynamics_remain_finite_without_contact(self):
        for _ in range(200):
            mujoco.mj_step(self.model, self.data)
        self.assertTrue(np.all(np.isfinite(self.data.qpos)))
        self.assertTrue(np.all(np.isfinite(self.data.qvel)))
        self.assertTrue(np.all(np.isfinite(self.data.sensordata)))

    def test_recorder_reads_fixed_peg_and_moving_hole_metadata(self):
        target_offset = np.array([0.010, 0.0, -0.020])
        fixture_body = self._id(
            mujoco.mjtObj.mjOBJ_BODY, "fixed_peg_fixture"
        )
        self.model.body_pos[fixture_body] += target_offset
        mujoco.mj_forward(self.model, self.data)

        with tempfile.TemporaryDirectory() as temp_dir:
            recorder = MujocoHDF5Recorder(
                model=self.model,
                data=self.data,
                output_dir=temp_dir,
                model_path=str(MODEL_PATH),
                record_images=False,
                enable_ft_tare=False,
                ee_body_name="hole_tool",
                task_name="moving_hole_fixed_peg",
                ft_force_sensor_name="hole_ft_force",
                ft_torque_sensor_name="hole_ft_torque",
                peg_geom_name="fixed_cylindrical_peg",
                peg_tip_site_name="fixed_peg_tip_site",
                hole_goal_site_name="hole_goal_site",
                sampled_target_site_name="fixed_peg_tip_site",
                hole_ring_geom_prefix="tool_hole_ring_",
                hole_axis_body=[0.0, 0.0, 1.0],
            )
            episode_path = recorder.start_episode(
                label="moving_hole_test",
                episode_metadata={
                    "target_offset_from_nominal_xyz": target_offset.tolist()
                },
            )
            recorder.stop_episode(status="manual_keep")

            with h5py.File(episode_path, "r") as h5:
                metadata = h5["episode_metadata"]
                self.assertEqual(
                    metadata.attrs["task_name"], "moving_hole_fixed_peg"
                )
                self.assertEqual(
                    metadata.attrs["sampled_target_site_name"],
                    "fixed_peg_tip_site",
                )
                np.testing.assert_allclose(
                    metadata["sampled_target_site_initial_pos_world"][()]
                    - metadata["sampled_target_site_nominal_pos_world"][()],
                    target_offset,
                )
                np.testing.assert_allclose(
                    metadata["hole_goal_site_initial_pos_world"][()],
                    metadata["hole_goal_site_nominal_pos_world"][()],
                )
                self.assertAlmostEqual(float(metadata.attrs["peg_radius_m"]), 0.011)
                self.assertAlmostEqual(float(metadata.attrs["peg_length_m"]), 0.045)
                self.assertAlmostEqual(float(metadata.attrs["hole_radius_m"]), 0.014)
                self.assertAlmostEqual(float(metadata.attrs["hole_depth_m"]), 0.045)
                self.assertEqual(int(metadata.attrs["hole_ring_segment_count"]), 24)

    def test_runtime_controller_accepts_role_and_sensor_names(self):
        from vptele.arm_control.robot_controller_mujoco_peg_tool_contact import (
            RobotControllerMuJoCoPegTool,
        )

        config = build_controller_config(load_mujoco_config(str(CONFIG_PATH)))
        config.update(
            {
                "auto_start": False,
                "launch_viewer": False,
                "show_camera_streams": False,
                "record_hdf5": False,
                "record_data": False,
                "enable_ros_interfaces": False,
                "visionpro_video_enabled": False,
            }
        )
        controller = RobotControllerMuJoCoPegTool(str(MODEL_PATH), config=config)

        self.assertEqual(controller.ft_force_sensor_name, "hole_ft_force")
        self.assertEqual(controller.ft_torque_sensor_name, "hole_ft_torque")
        self.assertEqual(controller.task_peg_geom_name, "fixed_cylindrical_peg")
        self.assertEqual(controller.task_moving_site_name, "hole_goal_site")
        self.assertEqual(controller.task_target_site_name, "fixed_peg_tip_site")
        self.assertEqual(controller.hole_body_name, "fixed_peg_fixture")
        self.assertEqual(len(controller.get_peg_ft_sensor()), 6)


if __name__ == "__main__":
    unittest.main()
