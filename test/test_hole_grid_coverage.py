#!/usr/bin/env python3

import tempfile
import unittest
from pathlib import Path

import h5py
import numpy as np

from scripts.check_hdf5_episode_quality import summarize_hole_grid_coverage


class HoleGridCoverageTest(unittest.TestCase):
    def _write_episode(
        self,
        root: Path,
        name: str,
        row: int,
        col: int,
        events=(),
        status="manual_keep",
    ):
        episode_dir = root / name
        episode_dir.mkdir()
        with h5py.File(episode_dir / "episode.hdf5", "w") as h5:
            metadata = h5.create_group("episode_metadata")
            metadata.attrs["hole_sampling_mode"] = "grid"
            metadata.attrs["hole_grid_rows"] = 5
            metadata.attrs["hole_grid_cols"] = 5
            metadata.attrs["hole_grid_cycle"] = 1
            metadata.attrs["hole_grid_index"] = 1
            metadata.attrs["hole_grid_row"] = row
            metadata.attrs["hole_grid_col"] = col
            metadata.attrs["hole_grid_cell_label"] = f"R{row}C{col}"
            metadata.attrs["hole_grid_sample_mode"] = "center"
            metadata.attrs["hole_grid_traversal_order"] = "shuffled"
            metadata.attrs["hole_grid_seed"] = 42
            metadata.attrs["status"] = status
            event_group = h5.create_group("events")
            event_group.create_dataset(
                "names",
                data=np.asarray(events, dtype=object),
                dtype=h5py.string_dtype(encoding="utf-8"),
            )

    def test_coverage_matrix_and_alarm_counts(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            self._write_episode(
                root,
                "episode_1",
                1,
                1,
                events=("task_success_site_reached",),
            )
            self._write_episode(
                root,
                "episode_2",
                1,
                1,
                events=("joint_torque_over_limit", "ft_wrench_over_limit"),
            )
            self._write_episode(root, "episode_3", 5, 5)
            self._write_episode(
                root,
                "discarded_episode",
                2,
                2,
                status="manual_discard",
            )

            report = summarize_hole_grid_coverage(root, target_per_cell=2)

        self.assertEqual(report["grid_episodes"], 4)
        self.assertEqual(report["counted_grid_episodes"], 3)
        self.assertEqual(report["counts"][0][0], 2)
        self.assertEqual(report["counts"][4][4], 1)
        self.assertEqual(report["counts"][1][1], 0)
        self.assertEqual(report["task_success_counts"][0][0], 1)
        self.assertEqual(report["task_success_rates"][0][0], 0.5)
        self.assertEqual(report["joint_torque_alarm_counts"][0][0], 1)
        self.assertEqual(report["ft_wrench_alarm_counts"][0][0], 1)
        self.assertNotIn("R1C1", report["under_target_cells"])
        self.assertIn("R5C5", report["under_target_cells"])


if __name__ == "__main__":
    unittest.main()
