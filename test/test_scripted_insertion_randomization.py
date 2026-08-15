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
    def test_role_mapping_uses_moving_hole_and_fixed_peg_sites(self):
        rc = Mock()
        rc.arm_sign = [1] * 7
        controller = ScriptedPegInsertionController(
            robot_controller=rc,
            ik_service_proxy=None,
            config={
                "moving_site_name": "hole_goal_site",
                "target_goal_site_name": "fixed_peg_tip_site",
                "target_approach_site_name": "fixed_peg_approach_goal_site",
                "target_body_name": "fixed_peg_fixture",
                "initial_robot_pose": [0, 0, 0, 0, 0, 0],
            },
        )
        positions = {
            "hole_goal_site": [1.0, 2.0, 3.0],
            "fixed_peg_tip_site": [4.0, 5.0, 6.0],
            "fixed_peg_approach_goal_site": [4.0, 5.045, 6.0],
        }
        rc.get_site_position.side_effect = positions.get

        np.testing.assert_allclose(controller._peg_w(), positions["hole_goal_site"])
        np.testing.assert_allclose(
            controller._hole_goal(), positions["fixed_peg_tip_site"]
        )
        np.testing.assert_allclose(
            controller._hole_entrance(),
            positions["fixed_peg_approach_goal_site"],
        )

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
