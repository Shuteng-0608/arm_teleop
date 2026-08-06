#!/usr/bin/env python3

import json
import tempfile
import unittest
from pathlib import Path

import h5py
import mujoco
import numpy as np

from vptele.utils.hole_grid_scheduler import HoleGridScheduler
from vptele.utils.mujoco_hdf5_recorder import MujocoHDF5Recorder


REPO_ROOT = Path(__file__).resolve().parents[1]
MODEL_PATH = REPO_ROOT / "model" / "pangu_all_right.xml"


class HoleGridMujocoIntegrationTest(unittest.TestCase):
    def setUp(self):
        self.model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
        self.data = mujoco.MjData(self.model)
        mujoco.mj_forward(self.model, self.data)

    def test_wall_body_translation_moves_task_sites_by_same_offset(self):
        body_id = mujoco.mj_name2id(
            self.model,
            mujoco.mjtObj.mjOBJ_BODY,
            "wall_task",
        )
        site_names = ["hole_goal_site"]
        site_ids = [
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, name)
            for name in site_names
        ]
        self.assertNotEqual(body_id, -1)
        self.assertNotIn(-1, site_ids)

        before = [self.data.site_xpos[site_id].copy() for site_id in site_ids]
        offset = np.array([0.024, 0.0, -0.048])
        self.model.body_pos[body_id] += offset
        mujoco.mj_forward(self.model, self.data)

        for site_id, old_position in zip(site_ids, before):
            np.testing.assert_allclose(
                self.data.site_xpos[site_id] - old_position,
                offset,
                atol=1e-12,
            )

    def test_hdf5_records_grid_assignment_and_sidecar_context(self):
        scheduler = HoleGridScheduler(
            traversal_order="row_major",
            sample_mode="center",
            seed=42,
        )
        context = scheduler.current()
        context.update(
            {
                "enabled": True,
                "hole_body_name": "wall_task",
                "hole_nominal_body_pos": [-0.25, -0.5, 1.0],
                "hole_actual_body_pos": [-0.298, -0.5, 1.048],
                "hole_offset_from_nominal_xyz": [-0.048, 0.0, 0.048],
            }
        )

        wall_body_id = mujoco.mj_name2id(
            self.model,
            mujoco.mjtObj.mjOBJ_BODY,
            "wall_task",
        )
        self.model.body_pos[wall_body_id] += np.array([-0.048, 0.0, 0.048])
        mujoco.mj_forward(self.model, self.data)

        with tempfile.TemporaryDirectory() as temp_dir:
            recorder = MujocoHDF5Recorder(
                model=self.model,
                data=self.data,
                output_dir=temp_dir,
                model_path=str(MODEL_PATH),
                record_images=False,
                enable_ft_tare=False,
                hole_center_site_name="hole_goal_site",
            )
            episode_path = recorder.start_episode(
                label="grid_test",
                episode_metadata=context,
            )
            recorder.stop_episode(status="manual_keep")

            with h5py.File(episode_path, "r") as h5:
                metadata = h5["episode_metadata"]
                self.assertEqual(metadata.attrs["hole_sampling_mode"], "grid")
                self.assertEqual(metadata.attrs["hole_grid_cell_label"], "R1C1")
                self.assertEqual(int(metadata.attrs["hole_grid_cycle"]), 1)
                self.assertEqual(int(metadata.attrs["hole_grid_seed"]), 42)
                self.assertAlmostEqual(float(metadata.attrs["peg_radius_m"]), 0.011)
                self.assertAlmostEqual(float(metadata.attrs["peg_length_m"]), 0.090)
                self.assertAlmostEqual(float(metadata.attrs["hole_radius_m"]), 0.014)
                self.assertAlmostEqual(float(metadata.attrs["hole_depth_m"]), 0.045)
                self.assertEqual(
                    int(metadata.attrs["hole_ring_segment_count"]),
                    24,
                )
                np.testing.assert_allclose(
                    metadata["hole_actual_offset_xyz"][()],
                    [-0.048, 0.0, 0.048],
                )
                np.testing.assert_allclose(
                    metadata["hole_offset_from_nominal_xyz"][()],
                    [-0.048, 0.0, 0.048],
                )
                np.testing.assert_allclose(
                    metadata["hole_goal_site_initial_pos_world"][()],
                    [-0.298, -0.521, 1.048],
                    atol=1e-12,
                )
                np.testing.assert_allclose(
                    metadata["hole_goal_site_nominal_pos_world"][()],
                    [-0.25, -0.521, 1.0],
                    atol=1e-12,
                )
                self.assertTrue(
                    np.all(
                        np.isfinite(
                            metadata["peg_tip_site_initial_pos_world"][()]
                        )
                    )
                )
                self.assertTrue(
                    np.all(np.isfinite(metadata["initial_hole_center_pos"][()]))
                )
                np.testing.assert_allclose(
                    metadata["initial_hole_goal_pos"][()],
                    metadata["hole_goal_site_initial_pos_world"][()],
                )

            sidecar_path = episode_path.parent / "metadata.json"
            sidecar = json.loads(sidecar_path.read_text(encoding="utf-8"))
            self.assertEqual(
                sidecar["episode_context"]["hole_grid_cell_label"],
                "R1C1",
            )
            self.assertAlmostEqual(
                sidecar["episode_context"]["peg_radius_m"],
                0.011,
            )
            self.assertAlmostEqual(
                sidecar["episode_context"]["hole_radius_m"],
                0.014,
            )
            self.assertEqual(
                sidecar["episode_context"]["hole_offset_from_nominal_xyz"],
                [-0.048, 0.0, 0.048],
            )


if __name__ == "__main__":
    unittest.main()
