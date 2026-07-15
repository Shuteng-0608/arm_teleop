from pathlib import Path
import tempfile
import unittest


try:
    import h5py
    import mujoco
    import numpy as np

    from vptele.utils.mujoco_hdf5_recorder import MujocoHDF5Recorder
except ImportError:
    h5py = None
    mujoco = None
    np = None
    MujocoHDF5Recorder = None


ROOT = Path(__file__).resolve().parents[1]
MODEL_PATH = ROOT / "model" / "pangu_all_right.xml"


class _ControllerSnapshot:
    def __init__(self, model, data):
        self.model = model
        self.data = data

    def _capture_render_state_locked(self):
        return {
            "time": np.asarray(float(self.data.time), dtype=np.float64),
            "qpos": self.data.qpos.copy(),
            "qvel": self.data.qvel.copy(),
            "act": self.data.act.copy(),
            "mocap_pos": self.data.mocap_pos.copy(),
            "mocap_quat": self.data.mocap_quat.copy(),
            "body_pos": self.model.body_pos.copy(),
            "body_quat": self.model.body_quat.copy(),
            "geom_rgba": self.model.geom_rgba.copy(),
            "mat_rgba": self.model.mat_rgba.copy(),
        }


@unittest.skipIf(mujoco is None or h5py is None, "MuJoCo/h5py not installed")
class MujocoHDF5RecorderIntegrationTest(unittest.TestCase):
    def test_async_recording_and_task_metadata(self):
        model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        controller = _ControllerSnapshot(model, data)

        with tempfile.TemporaryDirectory() as output_dir:
            recorder = MujocoHDF5Recorder(
                model=model,
                data=data,
                output_dir=output_dir,
                model_path=str(MODEL_PATH),
                force_hz=500.0,
                state_hz=30.0,
                image_hz=30.0,
                record_images=True,
                camera_names=["ee_cam", "base_top_cam"],
                image_width=640,
                image_height=480,
                image_compression=None,
                async_io=True,
                async_queue_size=256,
                write_batch_size=32,
                hole_center_site_name="hole_goal_site",
                task_success_metadata={
                    "task_success_condition_type": "site_distance_dwell",
                    "task_success_distance_threshold_m": 0.008,
                    "task_success_dwell_time_s": 0.10,
                },
            )

            hdf5_path = recorder.start_episode("integration")
            for _ in range(40):
                mujoco.mj_step(model, data)
                recorder.record_if_needed(controller)
            recorder.stop_episode("test_complete")

            self.assertIsNotNone(hdf5_path)
            with h5py.File(hdf5_path, "r") as h5_file:
                self.assertGreater(h5_file.attrs["n_force"], 0)
                self.assertGreater(h5_file.attrs["n_state"], 0)
                self.assertGreater(h5_file.attrs["n_image"], 0)
                self.assertEqual(
                    h5_file["observations/images/ee_cam"].shape[0],
                    h5_file.attrs["n_image"],
                )
                self.assertEqual(
                    h5_file["observations/images/ee_cam"].shape[1:],
                    (480, 640, 3),
                )
                self.assertEqual(
                    h5_file["observations/images/base_top_cam"].shape[0],
                    h5_file.attrs["n_image"],
                )

                task = h5_file["episode_metadata/task"]
                self.assertAlmostEqual(task.attrs["peg_radius_m"], 0.010)
                self.assertAlmostEqual(task.attrs["peg_length_m"], 0.090)
                self.assertAlmostEqual(task.attrs["hole_inner_radius_m"], 0.014)
                self.assertAlmostEqual(task.attrs["hole_depth_m"], 0.023487)
                self.assertAlmostEqual(
                    task.attrs["task_success_distance_threshold_m"],
                    0.008,
                )
                self.assertEqual(
                    task["hole_goal_site_pos_local"][:].tolist(),
                    [0.0, -0.016, 0.0],
                )


if __name__ == "__main__":
    unittest.main()
