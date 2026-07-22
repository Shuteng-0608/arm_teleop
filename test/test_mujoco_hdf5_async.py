#!/usr/bin/env python3

import json
import tempfile
import unittest
from pathlib import Path

import h5py
import mujoco
import numpy as np

from vptele.utils.mujoco_hdf5_recorder import MujocoHDF5Recorder


REPO_ROOT = Path(__file__).resolve().parents[1]
MODEL_PATH = REPO_ROOT / "model" / "pangu_all_right.xml"


class AsyncHDF5RecorderTest(unittest.TestCase):
    def setUp(self):
        self.model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
        self.data = mujoco.MjData(self.model)
        mujoco.mj_forward(self.model, self.data)

    def test_numeric_snapshots_are_drained_and_trimmed_on_stop(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            recorder = MujocoHDF5Recorder(
                model=self.model,
                data=self.data,
                output_dir=temp_dir,
                record_images=False,
                enable_ft_tare=False,
                force_hz=500.0,
                state_hz=30.0,
            )
            episode_path = recorder.start_episode(label="async_numeric")

            expected = {
                "force_t": [],
                "force_episode_t": [],
                "raw": [],
                "gravity": [],
                "compensated": [],
                "state_t": [],
                "state_episode_t": [],
                "ee_pose": [],
                "joint_pos": [],
                "joint_vel": [],
                "joint_torque": [],
                "joint_pos_command": [],
            }

            # Cross both batch thresholds and leave partial batches for the
            # stop barrier to drain.
            for _ in range(700):
                mujoco.mj_step(self.model, self.data)
                t_sim = float(self.data.time)
                t_episode = t_sim - recorder.episode_start_sim_time

                if t_sim + 1e-12 >= recorder.next_force_t:
                    raw = recorder._ft_wrench_raw()
                    gravity = recorder._ft_gravity_wrench()
                    expected["force_t"].append(t_sim)
                    expected["force_episode_t"].append(t_episode)
                    expected["raw"].append(raw.copy())
                    expected["gravity"].append(gravity.copy())
                    expected["compensated"].append(
                        recorder._compensate_ft_wrench(raw, gravity).copy()
                    )

                if t_sim + 1e-12 >= recorder.next_state_t:
                    expected["state_t"].append(t_sim)
                    expected["state_episode_t"].append(t_episode)
                    expected["ee_pose"].append(recorder._ee_pose().copy())
                    expected["joint_pos"].append(recorder._joint_pos().copy())
                    expected["joint_vel"].append(recorder._joint_vel().copy())
                    expected["joint_torque"].append(
                        recorder._joint_torque().copy()
                    )
                    expected["joint_pos_command"].append(
                        recorder._joint_pos_command().copy()
                    )

                recorder.record_if_needed(None)

            recorder.stop_episode(status="test_complete")
            recorder.close()

            with h5py.File(episode_path, "r") as h5:
                n_force = int(h5["episode_metadata"].attrs["n_force"])
                n_state = int(h5["episode_metadata"].attrs["n_state"])
                self.assertGreater(n_force, 0)
                self.assertGreater(n_state, 0)
                self.assertEqual(h5["timestamps/force"].shape, (n_force,))
                self.assertEqual(h5["timestamps/state"].shape, (n_state,))
                self.assertEqual(
                    h5["observations/ft_wrench"].shape,
                    (n_force, 6),
                )
                self.assertEqual(
                    h5["observations/joint_pos"].shape,
                    (n_state, 7),
                )
                self.assertTrue(
                    np.all(np.diff(h5["timestamps/force"][()]) > 0.0)
                )
                expected_paths = {
                    "timestamps/force": "force_t",
                    "timestamps/force_episode": "force_episode_t",
                    "observations/ft_wrench_raw": "raw",
                    "observations/ft_wrench_gravity": "gravity",
                    "observations/ft_wrench": "compensated",
                    "timestamps/state": "state_t",
                    "timestamps/state_episode": "state_episode_t",
                    "observations/ee_pose": "ee_pose",
                    "observations/joint_pos": "joint_pos",
                    "observations/joint_vel": "joint_vel",
                    "observations/joint_torque": "joint_torque",
                    "actions/joint_pos_command": "joint_pos_command",
                    "action": "joint_pos_command",
                }
                for path, expected_key in expected_paths.items():
                    self.assertEqual(h5[path].dtype, np.dtype(np.float64))
                    np.testing.assert_array_equal(
                        h5[path][()],
                        np.asarray(expected[expected_key], dtype=np.float64),
                    )

    def test_rendered_frames_keep_requested_simulation_timestamps(self):
        width = 32
        height = 24
        with tempfile.TemporaryDirectory() as temp_dir:
            recorder = MujocoHDF5Recorder(
                model=self.model,
                data=self.data,
                output_dir=temp_dir,
                record_images=True,
                camera_names=["ee_cam"],
                image_width=width,
                image_height=height,
                image_hz=30.0,
                enable_ft_tare=False,
            )
            episode_path = recorder.start_episode(label="async_image")
            requested_times = []

            for _ in range(40):
                mujoco.mj_step(self.model, self.data)
                request = recorder.record_if_needed(None)
                if request is None:
                    continue
                requested_times.append(request.t_sim)
                frame = np.full(
                    (height, width, 3),
                    fill_value=len(requested_times),
                    dtype=np.uint8,
                )
                self.assertTrue(
                    recorder.enqueue_image_sample(request, {"ee_cam": frame})
                )

            recorder.stop_episode(status="test_complete")
            recorder.close()

            with h5py.File(episode_path, "r") as h5:
                recorded_times = h5["timestamps/image"][()]
                np.testing.assert_allclose(recorded_times, requested_times)
                self.assertEqual(
                    h5["observations/images/ee_cam"].shape,
                    (len(requested_times), height, width, 3),
                )
                self.assertEqual(
                    h5["observations/images/ee_cam"].dtype,
                    np.dtype(np.uint8),
                )
                for index in range(len(requested_times)):
                    self.assertTrue(
                        np.all(
                            h5["observations/images/ee_cam"][index]
                            == index + 1
                        )
                    )
                self.assertEqual(
                    int(
                        h5["episode_metadata"].attrs[
                            "dropped_image_samples"
                        ]
                    ),
                    0,
                )

    def test_dropped_image_request_is_persisted_without_blocking_stop(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            recorder = MujocoHDF5Recorder(
                model=self.model,
                data=self.data,
                output_dir=temp_dir,
                record_images=True,
                camera_names=["ee_cam"],
                image_width=32,
                image_height=24,
                enable_ft_tare=False,
            )
            episode_path = recorder.start_episode(label="dropped_image")
            mujoco.mj_step(self.model, self.data)
            request = recorder.record_if_needed(None)
            self.assertIsNotNone(request)
            recorder.drop_image_request(request, "test_queue_full")
            recorder.stop_episode(status="test_complete")
            recorder.close()

            with h5py.File(episode_path, "r") as h5:
                self.assertEqual(int(h5.attrs["n_image"]), 0)
                self.assertEqual(int(h5.attrs["dropped_image_samples"]), 1)
                event_names = [
                    value.decode() if isinstance(value, bytes) else value
                    for value in h5["events/names"][()]
                ]
                drop_index = event_names.index("image_dropped")
                extra_value = h5["events/extra_json"][drop_index]
                if isinstance(extra_value, bytes):
                    extra_value = extra_value.decode()
                self.assertEqual(
                    json.loads(extra_value)["reason"],
                    "test_queue_full",
                )


if __name__ == "__main__":
    unittest.main()
