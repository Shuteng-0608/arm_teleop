import unittest

from vptele.utils.in_hole_disturbance_scheduler import (
    InHoleDisturbanceScheduler,
)


class InHoleDisturbanceSchedulerTest(unittest.TestCase):
    def test_cycle_covers_every_cell_once(self):
        scheduler = InHoleDisturbanceScheduler(
            [0.30, 0.55, 0.80],
            8,
            [3.4, 3.8],
            order="shuffled",
            seed=73,
        )
        samples = [scheduler.take() for _ in range(scheduler.size)]
        cells = {
            (sample.depth_index, sample.direction_index, sample.amplitude_index)
            for sample in samples
        }
        self.assertEqual(scheduler.size, 48)
        self.assertEqual(len(cells), 48)
        self.assertEqual((scheduler.cycle, scheduler.index), (1, 0))

    def test_seeded_order_is_reproducible(self):
        kwargs = dict(
            depth_fractions=[0.25, 0.75],
            direction_bins=4,
            amplitudes_mm=[3.5],
            order="shuffled",
            seed=9,
        )
        first = InHoleDisturbanceScheduler(**kwargs)
        second = InHoleDisturbanceScheduler(**kwargs)
        self.assertEqual(
            [first.take().to_dict() for _ in range(first.size)],
            [second.take().to_dict() for _ in range(second.size)],
        )

    def test_row_major_cursor(self):
        scheduler = InHoleDisturbanceScheduler(
            [0.3, 0.6],
            4,
            [3.2, 3.8],
            order="row_major",
            start_cycle=2,
            start_index=11,
        )
        sample = scheduler.current()
        self.assertEqual(sample.cycle, 2)
        self.assertEqual(sample.depth_index, 1)
        self.assertEqual(sample.direction_index, 1)
        self.assertEqual(sample.amplitude_index, 1)
        self.assertEqual(sample.direction_deg, 90.0)


if __name__ == "__main__":
    unittest.main()
