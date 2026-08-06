#!/usr/bin/env python3

import sys
import unittest
from pathlib import Path

import numpy as np


VPTELE_DIR = Path(__file__).resolve().parents[1] / "vptele"
if str(VPTELE_DIR) not in sys.path:
    sys.path.insert(0, str(VPTELE_DIR))

from arm_control.arm_teleop_mujoco_marvin import (
    _first_hand_transform,
    vendor_joints_to_controller_command,
)


class MarvinTeleopMathTest(unittest.TestCase):
    def test_vendor_solution_round_trips_through_controller_signs(self):
        vendor_joints = np.array(
            [-1.57, -1.57, 1.57, -1.6, -1.57, 0.0, 0.0]
        )
        arm_sign = np.array([1.0, -1.0, 1.0, -1.0, 1.0, 1.0, 1.0])

        command = vendor_joints_to_controller_command(vendor_joints, arm_sign)

        np.testing.assert_allclose(
            command,
            [-1.57, 1.57, 1.57, 1.6, -1.57, 0.0, 0.0],
        )
        np.testing.assert_allclose(command * arm_sign, vendor_joints)

    def test_vendor_conversion_rejects_zero_sign(self):
        with self.assertRaisesRegex(ValueError, "cannot contain zero"):
            vendor_joints_to_controller_command(
                np.zeros(7),
                [1.0, -1.0, 1.0, 0.0, 1.0, 1.0, 1.0],
            )

    def test_first_hand_transform_accepts_single_or_batched_matrix(self):
        matrix = np.eye(4)
        np.testing.assert_allclose(_first_hand_transform(matrix), matrix)
        np.testing.assert_allclose(
            _first_hand_transform(np.stack([matrix, matrix])),
            matrix,
        )
        self.assertIsNone(_first_hand_transform(np.zeros(3)))


if __name__ == "__main__":
    unittest.main()
