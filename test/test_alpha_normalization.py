import math
import sys
import unittest
from pathlib import Path


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "vptele"))

from mh6_palm_solver.alpha_normalization import (  # noqa: E402
    AlphaRange,
    alpha_to_signed_normalized,
    signed_normalized_to_alpha,
)


class AlphaNormalizationTest(unittest.TestCase):
    def test_three_calibration_points(self):
        args = (-20.0, 5.0, 45.0)
        self.assertEqual(alpha_to_signed_normalized(-20.0, *args), -1.0)
        self.assertEqual(alpha_to_signed_normalized(5.0, *args), 0.0)
        self.assertEqual(alpha_to_signed_normalized(45.0, *args), 1.0)

    def test_asymmetric_round_trip(self):
        args = (-90.8, 0.0, 31.1)
        for alpha in (-90.8, -40.0, 0.0, 10.0, 31.1):
            signed = alpha_to_signed_normalized(alpha, *args, clip=False)
            restored = signed_normalized_to_alpha(signed, *args, clip=False)
            self.assertAlmostEqual(restored, alpha)

    def test_clip_modes(self):
        self.assertEqual(
            alpha_to_signed_normalized(-30.0, -20.0, 0.0, 10.0, clip=True),
            -1.0,
        )
        self.assertEqual(
            signed_normalized_to_alpha(2.0, -20.0, 0.0, 10.0, clip=True),
            10.0,
        )
        with self.assertRaises(ValueError):
            alpha_to_signed_normalized(
                -30.0,
                -20.0,
                0.0,
                10.0,
                clip=False,
            )
        with self.assertRaises(ValueError):
            signed_normalized_to_alpha(
                2.0,
                -20.0,
                0.0,
                10.0,
                clip=False,
            )

    def test_non_finite_and_invalid_range_rejected(self):
        for value in (math.nan, math.inf, -math.inf):
            with self.assertRaises(ValueError):
                alpha_to_signed_normalized(value, -1.0, 0.0, 1.0)
        with self.assertRaises(ValueError):
            AlphaRange(-1.0, 0.0, 0.0)


if __name__ == "__main__":
    unittest.main()
