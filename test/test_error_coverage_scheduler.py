import unittest

from vptele.utils.error_coverage_scheduler import RimContactCoverageScheduler


class RimContactCoverageSchedulerTest(unittest.TestCase):
    def test_cycle_covers_every_radius_angle_pair_once(self):
        scheduler = RimContactCoverageScheduler(
            [4.0, 6.0, 8.0, 10.0],
            24,
            order="shuffled",
            seed=42,
            angle_jitter_deg=5.0,
        )
        samples = [scheduler.take() for _ in range(scheduler.size)]
        pairs = {(sample.radius_index, sample.angle_index) for sample in samples}
        self.assertEqual(len(samples), 96)
        self.assertEqual(len(pairs), 96)
        self.assertEqual(scheduler.cycle, 1)
        self.assertEqual(scheduler.index, 0)
        for sample in samples:
            delta = ((sample.angle_deg - sample.nominal_angle_deg + 180) % 360) - 180
            self.assertLessEqual(abs(delta), 5.0)

    def test_seeded_order_and_jitter_are_reproducible(self):
        kwargs = dict(
            radii_mm=[4.0, 8.0],
            angle_bins=8,
            order="shuffled",
            seed=7,
            angle_jitter_deg=3.0,
        )
        first = RimContactCoverageScheduler(**kwargs)
        second = RimContactCoverageScheduler(**kwargs)
        self.assertEqual(
            [first.take().to_dict() for _ in range(first.size)],
            [second.take().to_dict() for _ in range(second.size)],
        )

    def test_row_major_resume_cursor(self):
        scheduler = RimContactCoverageScheduler(
            [4.0, 6.0],
            4,
            order="row_major",
            start_cycle=2,
            start_index=5,
        )
        sample = scheduler.current()
        self.assertEqual(sample.cycle, 2)
        self.assertEqual(sample.radius_index, 1)
        self.assertEqual(sample.angle_index, 1)
        self.assertEqual(sample.nominal_angle_deg, 90.0)


if __name__ == "__main__":
    unittest.main()
