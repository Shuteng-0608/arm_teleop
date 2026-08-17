#!/usr/bin/env python3

import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np

from vptele.arm_control.scripted_insertion import (
    EpisodeResult,
    Phase,
    ScriptedPegInsertionController,
)


class ScriptedInsertionRandomizationTest(unittest.TestCase):
    def test_stratified_error_sampling_uses_configured_radius_angle_cells(self):
        rc = Mock()
        rc.arm_sign = [1] * 7
        controller = ScriptedPegInsertionController(
            robot_controller=rc,
            ik_service_proxy=None,
            config={
                "initial_robot_pose": [0, 0, 0, 0, 0, 0],
                "error_coverage_mode": "stratified_radius_angle",
                "rim_contact_radii_mm": [4.0, 8.0],
                "rim_contact_angle_bins": 4,
                "error_coverage_order": "row_major",
                "rim_contact_angle_jitter_deg": 0.0,
            },
        )

        samples = []
        for _ in range(8):
            controller._sample_error()
            samples.append(controller.get_last_error_info())

        self.assertEqual(
            [sample["scripted_error_xy_mm"] for sample in samples],
            [4.0] * 4 + [8.0] * 4,
        )
        self.assertEqual(
            [sample["scripted_error_angle_deg"] for sample in samples[:4]],
            [0.0, 90.0, 180.0, 270.0],
        )
        self.assertEqual(
            len({sample["scripted_error_cell_label"] for sample in samples}),
            8,
        )

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

    def test_approach_uses_feedback_speed_and_bounded_contact_detection(self):
        controller = ScriptedPegInsertionController.__new__(
            ScriptedPegInsertionController
        )
        controller.cfg = {
            "approach_standoff_m": 0.03,
            "approach_control_period_s": 0.03,
            "approach_force_poll_period_s": 0.005,
            "approach_free_speed_m_s": 0.05,
            "approach_far_speed_m_s": 0.025,
            "approach_near_speed_m_s": 0.008,
            "approach_probe_speed_m_s": 0.002,
            "approach_far_distance_m": 0.01,
            "approach_near_distance_m": 0.003,
            "approach_arrival_tolerance_m": 0.0005,
            "wall_contact_detect_force_n": 3.0,
            "wall_contact_detect_dwell_s": 0.005,
            "wall_contact_target_dwell_s": 0.005,
            "wall_contact_max_push_m": 0.0015,
            "contact_offset_tolerance_mm": 1.0,
        }
        controller._insert_axis = np.asarray([0.0, -1.0, 0.0])
        controller._err_w = np.asarray([0.004, 0.0, 0.0])
        controller._wall_threshold = 8.0
        controller._episode_metrics = {}
        controller._episode_deadline = None
        controller._arm_sign = [1] * 7
        controller.FORCE_LIMIT = 40.0
        current = np.asarray([0.0, 0.05, 0.0], dtype=float)
        desired = current.copy()

        class FakeRC:
            sim_timestep = 0.001

            def set_arm_positions(self, _joints):
                nonlocal current
                current = desired.copy()

        controller.rc = FakeRC()
        controller._hole_entrance = Mock(
            return_value=np.asarray([0.0, 0.0, 0.0])
        )
        controller._peg_w = Mock(side_effect=lambda: current.copy())
        controller._overload = Mock(return_value=False)
        controller._episode_timed_out = Mock(return_value=False)

        def solve(target, constrain_ori):
            nonlocal desired
            self.assertTrue(constrain_ori)
            desired = np.asarray(target, dtype=float).copy()
            return [0.0] * 7

        controller._mujoco_ik_step = Mock(side_effect=solve)
        controller._ft_world = Mock(
            side_effect=lambda: np.asarray(
                [10.0 if current[1] <= 0.0005 else 0.0, 0, 0, 0, 0, 0],
                dtype=float,
            )
        )

        with patch("vptele.arm_control.scripted_insertion.time.sleep"):
            result = controller._approach()

        self.assertIsNone(result)
        metrics = controller.get_episode_metrics()
        self.assertEqual(metrics["scripted_contact_detected"], 1)
        self.assertEqual(metrics["scripted_contact_target_reached"], 1)
        self.assertAlmostEqual(metrics["scripted_contact_target_offset_mm"], 4.0)
        self.assertLess(controller._mujoco_ik_step.call_count, 150)

    def test_force_world_requires_gravity_compensated_controller_api(self):
        controller = ScriptedPegInsertionController.__new__(
            ScriptedPegInsertionController
        )
        controller.rc = Mock()
        controller.rc.get_gravity_compensated_ft_wrench_world.return_value = [
            1.0,
            2.0,
            3.0,
            0.1,
            0.2,
            0.3,
        ]

        np.testing.assert_allclose(
            controller._ft_world(),
            [1.0, 2.0, 3.0, 0.1, 0.2, 0.3],
        )
        controller.rc.get_peg_ft_sensor.assert_not_called()

        del controller.rc.get_gravity_compensated_ft_wrench_world
        with self.assertRaisesRegex(RuntimeError, "gravity-compensated"):
            controller._ft_world()

    def test_in_hole_disturbance_recovers_with_lateral_compensated_force(self):
        controller = ScriptedPegInsertionController.__new__(
            ScriptedPegInsertionController
        )
        controller.cfg = {
            "in_hole_control_period_s": 0.01,
            "in_hole_disturbance_ramp_steps": 2,
            "in_hole_disturbance_hold_steps": 0,
            "in_hole_contact_detect_force_n": 2.0,
            "in_hole_contact_detect_dwell_s": 0.01,
            "in_hole_contact_detection_min_fraction": 0.70,
            "in_hole_contact_target_force_n": 5.0,
            "in_hole_contact_target_dwell_s": 0.01,
            "in_hole_contact_release_force_n": 1.0,
            "in_hole_contact_release_dwell_s": 0.01,
            "in_hole_force_limit_n": 25.0,
            "in_hole_force_filter_alpha": 1.0,
            "in_hole_correction_gain_m_per_n": 0.001,
            "in_hole_force_correction_sign": -1.0,
            "in_hole_correction_max_step_m": 0.001,
            "in_hole_correction_max_travel_m": 0.006,
            "in_hole_correction_timeout_s": 1.0,
            "in_hole_return_center_steps": 2,
        }
        controller._insert_axis = np.asarray([0.0, -1.0, 0.0])
        controller._in_hole_disturbance_sample = SimpleNamespace(
            depth_fraction=0.5,
            direction_deg=0.0,
            amplitude_mm=4.0,
        )
        controller._episode_metrics = {}
        controller._episode_deadline = None
        controller._arm_sign = [1] * 7
        controller._in_hole_phase = "insert"
        controller._in_hole_phase_code = 1
        controller._expert_action_mask = 1
        events = []
        controller._event_callback = lambda name, extra: events.append((name, extra))
        current = np.asarray([0.0, 0.0, 0.0], dtype=np.float64)
        desired = current.copy()

        class FakeRC:
            def set_arm_positions(self, _joints):
                nonlocal current
                current = desired.copy()

        controller.rc = FakeRC()
        controller._peg_w = Mock(side_effect=lambda: current.copy())
        controller._episode_timed_out = Mock(return_value=False)
        controller._freeze_current_arm = Mock()
        controller._smooth_move_ee = Mock(return_value=True)

        def solve(target, constrain_ori):
            nonlocal desired
            self.assertTrue(constrain_ori)
            desired = np.asarray(target, dtype=np.float64).copy()
            return [0.0] * 7

        controller._mujoco_ik_step = Mock(side_effect=solve)
        controller._ft_world = Mock(
            side_effect=lambda: np.asarray(
                [5.0 if current[0] >= 0.003 else 0.0, 0.0, 0.0, 0, 0, 0],
                dtype=np.float64,
            )
        )

        with patch("vptele.arm_control.scripted_insertion.time.sleep"):
            result = controller._run_in_hole_disturbance(
                nominal_w=np.asarray([0.0, -0.01, 0.0]),
                actual_depth_fraction=0.5,
            )

        self.assertIsNone(result)
        metrics = controller.get_episode_metrics()
        self.assertEqual(metrics["scripted_in_hole_contact_detected"], 1)
        self.assertEqual(metrics["scripted_in_hole_contact_target_reached"], 1)
        self.assertEqual(metrics["scripted_in_hole_recovery_success"], 1)
        self.assertGreater(metrics["scripted_in_hole_correction_travel_mm"], 0.0)
        self.assertIn("scripted_in_hole_disturbance", [row[0] for row in events])
        self.assertIn("scripted_in_hole_recovery", [row[0] for row in events])
        self.assertEqual(controller.get_control_annotation()["expert_action_mask"], 1)


if __name__ == "__main__":
    unittest.main()
