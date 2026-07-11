import sys
import unittest
from pathlib import Path


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "vptele"))

from mh6_palm_solver.alpha_normalization import AlphaRange  # noqa: E402
from mh6_palm_solver.solve_from_arpha2_arpha3_theta1 import (  # noqa: E402
    MH6PalmSolver,
)


class MH6PalmSolverSignedTest(unittest.TestCase):
    def setUp(self):
        self.solver = MH6PalmSolver()
        self.arpha1_range = AlphaRange(-23.6, 0.0, 10.0)

    def test_legacy_map_normalized_still_does_not_clip(self):
        result = self.solver.map_normalized(2.0, -1.0, 1.5)
        for actual, expected in zip(result, (212.7, 298.0, -39.85)):
            self.assertAlmostEqual(actual, expected)

    def test_safe_map_clip_modes(self):
        self.assertEqual(
            self.solver.map_normalized_safe(2.0, -1.0, 1.5, clip=True),
            self.solver.map_normalized(1.0, 0.0, 1.0),
        )
        with self.assertRaises(ValueError):
            self.solver.map_normalized_safe(2.0, 0.0, 0.0, clip=False)

    def test_arpha1_range_is_required_by_all_signed_solver_interfaces(self):
        with self.assertRaisesRegex(ValueError, "arpha1_range is required"):
            self.solver.solve_signed(47.0, -80.0, -20.0)
        with self.assertRaisesRegex(ValueError, "arpha1_range is required"):
            self.solver.solve_signed_from_normalized(0.641, 0.582, 0.885)
        with self.assertRaisesRegex(ValueError, "arpha1_range is required"):
            self.solver.solve_motor_via_signed(47.0, -80.0, -20.0)

    def test_signed_triplet_order_and_arpha2_star_sign(self):
        signed = self.solver.solve_signed(
            47.0,
            -80.0,
            -20.0,
            arpha1_range=self.arpha1_range,
            clip=False,
        )
        self.assertEqual(len(signed), 2)
        # arpha2_star=-47 lies on the negative segment [-90.8, 0].
        self.assertAlmostEqual(signed[0][1], -47.0 / 90.8)

    def test_motor_via_signed_matches_legacy_without_clipping(self):
        legacy = self.solver.solve_motor(47.0, -80.0, -20.0)
        via_signed = self.solver.solve_motor_via_signed(
            47.0,
            -80.0,
            -20.0,
            arpha1_range=self.arpha1_range,
            clip=False,
        )
        self.assertEqual(via_signed, legacy)

    def test_legacy_public_solver_outputs_are_unchanged(self):
        arpha = self.solver.solve_arpha(47.0, -80.0, -20.0)
        self.assertEqual(len(arpha), 2)
        self.assertAlmostEqual(arpha[0][0], -4.399, places=3)
        self.assertAlmostEqual(arpha[1][0], -19.060, places=3)
        self.assertEqual([solution[1:] for solution in arpha], [
            [-47.0, -80.0],
            [-47.0, -80.0],
        ])
        self.assertEqual(
            self.solver.solve_motor(47.0, -80.0, -20.0),
            [
                [481.5468, 303.304, 581.6667],
                [420.0444, 303.304, 581.6667],
            ],
        )


if __name__ == "__main__":
    unittest.main()
