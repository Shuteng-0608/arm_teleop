#!/usr/bin/env python3

import unittest

import numpy as np

from vptele.utils.hole_grid_scheduler import HoleGridScheduler


class HoleGridSchedulerTest(unittest.TestCase):
    def test_row_major_centers_cover_5x5_grid(self):
        scheduler = HoleGridScheduler(traversal_order="row_major", seed=7)
        samples = []
        for _ in range(25):
            samples.append(scheduler.current())
            scheduler.advance()

        self.assertEqual(
            len({sample["hole_grid_cell_label"] for sample in samples}),
            25,
        )
        self.assertEqual(samples[0]["hole_grid_cell_label"], "R1C1")
        self.assertEqual(samples[-1]["hole_grid_cell_label"], "R5C5")
        np.testing.assert_allclose(
            samples[0]["hole_actual_offset_xyz"],
            [-0.048, 0.0, 0.048],
        )
        np.testing.assert_allclose(
            samples[12]["hole_actual_offset_xyz"],
            [0.0, 0.0, 0.0],
            atol=1e-15,
        )
        np.testing.assert_allclose(
            samples[-1]["hole_actual_offset_xyz"],
            [0.048, 0.0, -0.048],
        )

    def test_shuffled_cycle_is_complete_and_reproducible(self):
        first = HoleGridScheduler(traversal_order="shuffled", seed=42)
        second = HoleGridScheduler(traversal_order="shuffled", seed=42)

        first_labels = []
        second_labels = []
        for _ in range(25):
            first_labels.append(first.current()["hole_grid_cell_label"])
            second_labels.append(second.current()["hole_grid_cell_label"])
            first.advance()
            second.advance()

        self.assertEqual(first_labels, second_labels)
        self.assertEqual(len(set(first_labels)), 25)

    def test_advance_wraps_to_next_cycle(self):
        scheduler = HoleGridScheduler(traversal_order="row_major", seed=1)
        for _ in range(25):
            scheduler.advance()

        current = scheduler.current()
        self.assertEqual(current["hole_grid_cycle"], 2)
        self.assertEqual(current["hole_grid_index"], 1)
        self.assertEqual(current["hole_grid_cell_label"], "R1C1")

    def test_current_is_stable_until_advanced(self):
        scheduler = HoleGridScheduler(
            sample_mode="uniform_in_cell",
            traversal_order="shuffled",
            seed=99,
        )
        first = scheduler.current()
        retry = scheduler.current()
        self.assertEqual(first, retry)

        x, _, z = first["hole_actual_offset_xyz"]
        self.assertLessEqual(first["hole_cell_x_bounds"][0], x)
        self.assertLessEqual(x, first["hole_cell_x_bounds"][1])
        self.assertLessEqual(first["hole_cell_z_bounds"][0], z)
        self.assertLessEqual(z, first["hole_cell_z_bounds"][1])

    def test_on_keep_policy_retries_discarded_cell(self):
        scheduler = HoleGridScheduler(traversal_order="row_major", seed=5)
        first = scheduler.current()
        retry = scheduler.complete_episode(keep=False, advance_policy="on_keep")
        self.assertEqual(retry, first)

        next_sample = scheduler.complete_episode(
            keep=True,
            advance_policy="on_keep",
        )
        self.assertEqual(next_sample["hole_grid_index"], 2)
        self.assertNotEqual(
            next_sample["hole_grid_cell_label"],
            first["hole_grid_cell_label"],
        )

    def test_resume_is_deterministic(self):
        running = HoleGridScheduler(seed=123, start_cycle=2, start_index=11)
        resumed = HoleGridScheduler(seed=123, start_cycle=2, start_index=11)
        self.assertEqual(running.current(), resumed.current())

    def test_invalid_configuration_is_rejected(self):
        with self.assertRaises(ValueError):
            HoleGridScheduler(rows=0)
        with self.assertRaises(ValueError):
            HoleGridScheduler(x_range=(0.1, -0.1))
        with self.assertRaises(ValueError):
            HoleGridScheduler(sample_mode="unknown")
        with self.assertRaises(ValueError):
            HoleGridScheduler(start_index=25)


if __name__ == "__main__":
    unittest.main()
