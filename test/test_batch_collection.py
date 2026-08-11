#!/usr/bin/env python3

import unittest

from vptele.utils.batch_collection import BatchProgress


class BatchProgressTest(unittest.TestCase):
    def test_rejections_do_not_advance_retained_target(self):
        progress = BatchProgress(target_kept=2, max_attempts=5)
        progress.begin_attempt()
        progress.register(False)
        self.assertEqual(progress.kept, 0)
        self.assertTrue(progress.can_attempt)

        progress.begin_attempt()
        progress.register(True)
        progress.begin_attempt()
        progress.register(True)
        self.assertTrue(progress.complete)
        self.assertFalse(progress.can_attempt)
        self.assertEqual(progress.attempted, 3)
        self.assertEqual(progress.kept, 2)

    def test_attempt_limit_stops_unreachable_batch(self):
        progress = BatchProgress(target_kept=2, max_attempts=2)
        for _ in range(2):
            progress.begin_attempt()
            progress.register(False)
        self.assertTrue(progress.exhausted)
        self.assertFalse(progress.complete)

    def test_consecutive_rejection_limit_resets_after_keep(self):
        progress = BatchProgress(
            target_kept=3,
            max_attempts=20,
            max_consecutive_rejections=2,
        )
        progress.begin_attempt()
        progress.register(False)
        progress.begin_attempt()
        progress.register(True)
        self.assertEqual(progress.consecutive_rejections, 0)
        progress.begin_attempt()
        progress.register(False)
        progress.begin_attempt()
        progress.register(False)
        self.assertTrue(progress.exhausted)


if __name__ == "__main__":
    unittest.main()
