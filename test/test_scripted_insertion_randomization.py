#!/usr/bin/env python3

import unittest
from unittest.mock import Mock

import numpy as np

from vptele.arm_control.scripted_insertion import (
    EpisodeResult,
    Phase,
    ScriptedPegInsertionController,
)


class ScriptedInsertionRandomizationTest(unittest.TestCase):
    def test_presampled_wall_threshold_is_not_resampled(self):
        controller = ScriptedPegInsertionController.__new__(
            ScriptedPegInsertionController
        )
        controller.cfg = {"episode_timeout_s": 1.0}
        controller._wall_threshold = 0.0
        controller._sample_wall_threshold = Mock()
        controller._init_dofs = Mock()
        controller._peg_w = Mock(return_value=np.zeros(3, dtype=np.float64))
        controller._peg_xmat = Mock(return_value=np.eye(3, dtype=np.float64))
        expected = EpisodeResult(False, "stop", Phase.APPROACH, 0, 0.0)
        controller._approach = Mock(return_value=expected)

        result = controller.run_episode(
            error_xy_mm=10.0,
            error_angle_deg=30.0,
            wall_threshold_n=23.5,
        )

        self.assertIs(result, expected)
        self.assertEqual(controller._wall_threshold, 23.5)
        controller._sample_wall_threshold.assert_not_called()


if __name__ == "__main__":
    unittest.main()
