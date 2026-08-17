import csv
import os
import tempfile
import unittest

import numpy as np

from core.right_teleop_playback import (
    load_teleop_trajectory,
    rounded_solver_state,
    validate_online_solution,
)


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

    def test_runtime_entry_uses_online_ik_not_precomputed_joints(self):
        script = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            "vptele",
            "playback_right_teleop.py",
        )
        with open(script, "r", encoding="utf-8") as input_file:
            source = input_file.read()
        self.assertIn('request.method = "redundancy_selector"', source)
        self.assertIn("service.call(request)", source)
        self.assertNotIn("load_redundancy_trajectory", source)
        self.assertNotIn("redundancy_q1", source)


if __name__ == "__main__":
    unittest.main()
