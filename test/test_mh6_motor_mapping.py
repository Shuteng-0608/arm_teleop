import sys
import unittest
from pathlib import Path


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "vptele"))

from mh6_palm_solver.alpha_normalization import AlphaRange  # noqa: E402
from mh6_palm_solver.motor_mapping import signed_to_motor_triplet  # noqa: E402


class MH6MotorMappingTest(unittest.TestCase):
    def setUp(self):
        self.arpha1_range = AlphaRange(-23.6, 0.0, 10.0)

    def test_arpha1_range_is_required(self):
        with self.assertRaisesRegex(ValueError, "arpha1_range is required"):
            signed_to_motor_triplet((0.0, 0.0, 0.0))

    def test_zero_triplet_uses_raw_arpha2_zero(self):
        self.assertEqual(
            signed_to_motor_triplet(
                (0.0, 0.0, 0.0),
                arpha1_range=self.arpha1_range,
                clip=False,
            ),
            [500.0, 500.0, 247.0],
        )

    def test_motor2_uses_negated_arpha2_star(self):
        # signed arpha2_star=-1 -> arpha2_star=-90.8 -> arpha2_raw=90.8.
        motors = signed_to_motor_triplet(
            (0.0, -1.0, 0.0),
            arpha1_range=self.arpha1_range,
            clip=False,
        )
        self.assertEqual(motors[1], 120.0)

    def test_motor_output_is_clamped(self):
        motors = signed_to_motor_triplet(
            (1.0, 0.0, 0.0),
            arpha1_range=AlphaRange(-23.6, 0.0, 200.0),
            clip=True,
        )
        self.assertEqual(motors[0], 1000.0)
        with self.assertRaises(ValueError):
            signed_to_motor_triplet(
                (1.0, 0.0, 0.0),
                arpha1_range=AlphaRange(-23.6, 0.0, 200.0),
                clip=False,
            )


if __name__ == "__main__":
    unittest.main()
