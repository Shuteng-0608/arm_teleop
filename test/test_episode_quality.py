#!/usr/bin/env python3

import tempfile
import unittest
from pathlib import Path

import h5py
import numpy as np

from vptele.utils.episode_quality import evaluate_episode_quality


def make_episode(path: Path, *, success=True, alarm=False, nan=False):
    with h5py.File(path, "w") as h5:
        observations = h5.create_group("observations")
        values = np.ones((12, 7), dtype=np.float64)
        if nan:
            values[0, 0] = np.nan
        observations.create_dataset("joint_pos", data=values)
        observations.create_dataset("joint_vel", data=np.zeros((12, 7)))
        observations.create_dataset("joint_torque", data=np.ones((12, 7)))
        observations.create_dataset("ee_pose", data=np.ones((12, 7)))
        observations.create_dataset("ft_wrench", data=np.ones((60, 6)))
        h5.create_dataset("action", data=np.ones((12, 7)))

        timestamps = h5.create_group("timestamps")
        timestamps.create_dataset("state_episode", data=np.arange(12) / 30.0)
        timestamps.create_dataset("force_episode", data=np.arange(60) / 500.0)

        event_names = ["record_start"]
        if success:
            event_names.append("task_success_site_reached")
            event_names.append("terminal_hold_start")
        if alarm:
            event_names.append("ft_wrench_over_limit")
        events = h5.create_group("events")
        string_dtype = h5py.string_dtype(encoding="utf-8")
        events.create_dataset("names", data=event_names, dtype=string_dtype)

        metadata = h5.create_group("episode_metadata")
        metadata.attrs["status"] = "scripted_success"
        metadata.attrs["async_error"] = ""
        metadata.attrs["dropped_numeric_samples"] = 0
        metadata.attrs["dropped_image_samples"] = 0
        metadata.attrs["n_state"] = 12
        metadata.attrs["n_force"] = 60
        metadata.attrs["n_image"] = 0


class EpisodeQualityTest(unittest.TestCase):
    def test_accepts_complete_successful_numeric_episode(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            path = Path(temp_dir) / "episode.hdf5"
            make_episode(path)
            decision = evaluate_episode_quality(
                path,
                execution_success=True,
                config={"require_images": False},
            )
            self.assertTrue(decision.accepted, decision.reasons)

    def test_rejects_missing_success_alarm_and_nan(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            path = Path(temp_dir) / "episode.hdf5"
            make_episode(path, success=False, alarm=True, nan=True)
            decision = evaluate_episode_quality(
                path,
                execution_success=True,
                config={"require_images": False},
            )
            self.assertFalse(decision.accepted)
            self.assertIn("task_success_event_missing", decision.reasons)
            self.assertIn("rejected_event:ft_wrench_over_limit", decision.reasons)
            self.assertIn(
                "nonfinite_dataset:observations/joint_pos",
                decision.reasons,
            )

    def test_execution_failure_is_always_rejected(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            path = Path(temp_dir) / "episode.hdf5"
            make_episode(path)
            decision = evaluate_episode_quality(
                path,
                execution_success=False,
                config={"require_images": False},
            )
            self.assertFalse(decision.accepted)
            self.assertIn("execution_failed", decision.reasons)


if __name__ == "__main__":
    unittest.main()
