import csv
import os
import tempfile
import unittest

import numpy as np
from scipy.spatial.transform import Rotation

from core.right_arm_trajectory import (
    INITIAL_RIGHT_ROBOT_POSE,
    RightArmTrajectoryMapper,
    euler_pose_to_quaternion_wxyz,
    euler_pose_to_quaternion_xyzw,
    load_right_wrist_frames,
    map_right_hand_to_robot_euler,
)


REFERENCE_ROTATION = np.array(
    [[0.0, 1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, 1.0]],
    dtype=float,
)


def transform(position):
    matrix = np.eye(4)
    matrix[:3, :3] = REFERENCE_ROTATION
    matrix[:3, 3] = position
    return matrix


class RightArmTrajectoryTest(unittest.TestCase):
    def test_reference_hand_pose_maps_to_initial_robot_pose(self):
        initial_hand = transform([0.2, 0.3, 0.4])
        mapped = map_right_hand_to_robot_euler(
            initial_hand, initial_hand[:3, 3]
        )
        np.testing.assert_allclose(
            mapped[:3], INITIAL_RIGHT_ROBOT_POSE[:3], atol=1e-12
        )
        np.testing.assert_allclose(
            Rotation.from_euler("XYZ", mapped[3:]).as_matrix(),
            Rotation.from_euler(
                "XYZ", INITIAL_RIGHT_ROBOT_POSE[3:]
            ).as_matrix(),
            atol=1e-12,
        )

    def test_position_axes_match_main_ros_mapping(self):
        initial_hand = transform([0.2, 0.3, 0.4])
        current_hand = transform([0.3, 0.5, 0.7])
        mapped = map_right_hand_to_robot_euler(
            current_hand, initial_hand[:3, 3]
        )
        expected = INITIAL_RIGHT_ROBOT_POSE.copy()
        expected[:3] += [0.2, 0.3, 0.1]
        np.testing.assert_allclose(mapped[:3], expected[:3], atol=1e-12)
        np.testing.assert_allclose(
            Rotation.from_euler("XYZ", mapped[3:]).as_matrix(),
            Rotation.from_euler("XYZ", expected[3:]).as_matrix(),
            atol=1e-12,
        )

    def test_quaternion_orders_are_explicit_and_equivalent(self):
        wxyz = euler_pose_to_quaternion_wxyz(INITIAL_RIGHT_ROBOT_POSE)
        xyzw = euler_pose_to_quaternion_xyzw(INITIAL_RIGHT_ROBOT_POSE)
        np.testing.assert_allclose(wxyz[:3], xyzw[:3], atol=0.0)
        np.testing.assert_allclose(wxyz[3:], xyzw[[6, 3, 4, 5]], atol=1e-12)
        self.assertAlmostEqual(float(np.linalg.norm(wxyz[3:])), 1.0, places=12)

    def test_mapper_calibrates_from_first_frame_position(self):
        first = transform([0.2, 0.3, 0.4])
        mapper = RightArmTrajectoryMapper(first)
        target = mapper.ik_target(first)
        np.testing.assert_allclose(target[:3], INITIAL_RIGHT_ROBOT_POSE[:3])

    def test_csv_loader_preserves_frames_and_rejects_nonmonotonic_time(self):
        header = ["timestamp"] + [
            "right_raw_matrix_{}{}".format(row, column)
            for row in range(4)
            for column in range(4)
        ]
        with tempfile.TemporaryDirectory() as directory:
            valid_path = os.path.join(directory, "valid.csv")
            with open(valid_path, "w", encoding="utf-8", newline="") as file_handle:
                writer = csv.DictWriter(file_handle, fieldnames=header)
                writer.writeheader()
                for timestamp in (0.0, 0.033):
                    matrix = transform([0.2 + timestamp, 0.3, 0.4])
                    row = {"timestamp": timestamp}
                    for matrix_row in range(4):
                        for matrix_column in range(4):
                            row[
                                "right_raw_matrix_{}{}".format(
                                    matrix_row, matrix_column
                                )
                            ] = matrix[matrix_row, matrix_column]
                    writer.writerow(row)
            frames = load_right_wrist_frames(valid_path)
            self.assertEqual([frame.index for frame in frames], [0, 1])
            self.assertEqual([frame.timestamp for frame in frames], [0.0, 0.033])

            invalid_path = os.path.join(directory, "invalid.csv")
            with open(invalid_path, "w", encoding="utf-8", newline="") as file_handle:
                writer = csv.DictWriter(file_handle, fieldnames=header)
                writer.writeheader()
                for _ in range(2):
                    matrix = transform([0.2, 0.3, 0.4])
                    row = {"timestamp": 0.0}
                    for matrix_row in range(4):
                        for matrix_column in range(4):
                            row[
                                "right_raw_matrix_{}{}".format(
                                    matrix_row, matrix_column
                                )
                            ] = matrix[matrix_row, matrix_column]
                    writer.writerow(row)
            with self.assertRaises(ValueError):
                load_right_wrist_frames(invalid_path)


if __name__ == "__main__":
    unittest.main()
