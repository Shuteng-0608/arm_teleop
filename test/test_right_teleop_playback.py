import csv
import os
import tempfile
import unittest
from types import SimpleNamespace
from unittest import mock

import numpy as np
from genpy import Time as RosTime

from core.right_teleop_playback import (
    load_teleop_trajectory,
    rounded_solver_state,
    validate_online_solution,
)
from playback_right_joint_trajectory import call_movej, make_message


MATRIX_FIELDS = [
    "right_raw_matrix_{}{}".format(row, column)
    for row in range(4)
    for column in range(4)
]


class RightTeleopPlaybackTest(unittest.TestCase):
    def source_file(self):
        temporary = tempfile.NamedTemporaryFile(
            mode="w", encoding="utf-8", newline="", delete=False
        )
        with temporary:
            writer = csv.DictWriter(
                temporary, fieldnames=["timestamp"] + MATRIX_FIELDS
            )
            writer.writeheader()
            for timestamp in (0.0, 0.1):
                transform = np.eye(4)
                transform[0, 3] = timestamp
                row = {"timestamp": timestamp}
                for matrix_row in range(4):
                    for matrix_column in range(4):
                        row[
                            "right_raw_matrix_{}{}".format(
                                matrix_row, matrix_column
                            )
                        ] = transform[matrix_row, matrix_column]
                writer.writerow(row)
        self.addCleanup(lambda: os.unlink(temporary.name))
        return temporary.name

    def test_raw_source_validation(self):
        frames, summary = load_teleop_trajectory(
            self.source_file(),
            expected_frame_count=2,
            expected_file_sha256=None,
        )
        self.assertEqual(len(frames), 2)
        self.assertAlmostEqual(summary.duration, 0.1)

    def test_online_solution_validation_and_rounding(self):
        previous = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]
        current = [0.02, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]
        joints, transition = validate_online_solution(
            current, previous, source_dt=0.1
        )
        self.assertAlmostEqual(transition.maximum_step, 0.02)
        self.assertAlmostEqual(transition.maximum_velocity, 0.2)
        self.assertEqual(rounded_solver_state(joints)[0], 0.02)

    def test_rejects_unsafe_online_step(self):
        previous = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]
        current = [0.04, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]
        with self.assertRaisesRegex(ValueError, "step"):
            validate_online_solution(current, previous, source_dt=0.1)

    def test_rejects_hardware_limit_violation(self):
        joints = [0.0, 0.0, 0.0, 2.5, 0.0, 0.0, 0.0]
        with self.assertRaisesRegex(ValueError, "outside"):
            validate_online_solution(joints)

    def test_command_contains_both_seven_joint_arms(self):
        right = [float(index) for index in range(7)]
        left = [float(-index) for index in range(7)]
        with mock.patch(
            "playback_right_joint_trajectory.rospy.Time.now",
            return_value=RosTime(),
        ):
            message = make_message(3, right, left, 0.0)
        self.assertEqual(list(message.right_arm.arm_joints), right)
        self.assertEqual(list(message.left_arm.arm_joints), left)
        self.assertEqual(message.right_arm.arm_id, 1)
        self.assertEqual(message.left_arm.arm_id, 0)

    def test_movej_helper_supports_main_ros_arm_order(self):
        service = mock.Mock()
        args = SimpleNamespace(movej_vel=0.5, movej_acc=5.0, movej_jerk=10.0)
        call_movej(service, [0.0] * 7, args, arm_id=0)
        request = service.call.call_args[0][0]
        self.assertEqual(request.arm_id, 0)
        self.assertEqual(request.vel, 0.5)
        self.assertEqual(request.acc, 5.0)
        self.assertEqual(request.jerk, 10.0)

    def test_runtime_entry_uses_online_ik_not_precomputed_joints(self):
        script = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            "vptele",
            "playback_right_teleop.py",
        )
        with open(script, "r", encoding="utf-8") as input_file:
            source = input_file.read()
        self.assertIn("request.method = method", source)
        self.assertIn('default="A1_minimum_jv"', source)
        self.assertIn("service.call(request)", source)
        self.assertNotIn("load_redundancy_trajectory", source)
        self.assertNotIn("redundancy_q1", source)
        self.assertIn('mode.add_argument("--preflight"', source)
        self.assertNotIn("--confirm-motion", source)
        self.assertIn(
            "else:\n        run_execute(args, input_path, frames, source_summary, mapper)",
            source,
        )
        self.assertIn("left_hold_joints = tuple(LEFT_HOME_JOINTS)", source)
        self.assertEqual(source.count("left_hold_joints,\n"), 3)
        self.assertIn('parser.add_argument("--movej-vel", type=float, default=0.5)', source)
        self.assertIn('parser.add_argument("--movej-acc", type=float, default=5.0)', source)
        self.assertIn('parser.add_argument("--movej-jerk", type=float, default=10.0)', source)
        self.assertIn(
            "call_movej(movej_service, solver.initial_joints, args, arm_id=1)", source
        )
        self.assertIn(
            "call_movej(movej_service, LEFT_HOME_JOINTS, args, arm_id=0)", source
        )
        self.assertIn("head_z_rotation = 0.0", source)
        self.assertNotIn("FeedbackService", source)
        self.assertNotIn("LogService", source)
        self.assertNotIn("call_feedback", source)
        self.assertNotIn("set_log", source)
        self.assertNotIn("set_teleop(teleop_service, False)", source)
        execute_source = source[
            source.index("def run_execute"):source.index("def main")
        ]
        self.assertLess(
            execute_source.index(
                "call_movej(movej_service, solver.initial_joints, args, arm_id=1)"
            ),
            execute_source.index(
                "call_movej(movej_service, LEFT_HOME_JOINTS, args, arm_id=0)"
            ),
        )
        self.assertLess(
            execute_source.index(
                "call_movej(movej_service, LEFT_HOME_JOINTS, args, arm_id=0)"
            ),
            execute_source.index("set_teleop(teleop_service, True)"),
        )
        self.assertLess(
            execute_source.index("set_teleop(teleop_service, True)"),
            execute_source.index("for frame in frames:"),
        )
        self.assertLess(
            execute_source.index("for frame in frames:"),
            execute_source.index("result = solver.solve(frame, target)"),
        )


if __name__ == "__main__":
    unittest.main()
