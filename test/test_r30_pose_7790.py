"""ROS-free deployment regression for the bundled trajectory and its IK seed."""
import ast
import os
from pathlib import Path
import unittest
from unittest.mock import patch

import numpy as np

from core.right_arm_trajectory import (
    R30_BALANCED_INITIAL_RIGHT_JOINTS,
    R30_BALANCED_INITIAL_RIGHT_ARM_ANGLE,
    RightArmTrajectoryMapper,
    default_right_trajectory_path,
)
from core.right_teleop_playback import load_teleop_trajectory, validate_online_solution

ROOT = Path(__file__).resolve().parents[1]
CSV = ROOT / "experiments/r30_pose_7790/R30_pose_7790_engineering.csv"


class R30Pose7790Test(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.frames, cls.summary = load_teleop_trajectory(
            str(CSV), expected_frame_count=772,
            expected_file_sha256="54410f14dbca6e609809c8dea3378ebba82a700db7e7d5efe25bc69a1ddbd91a",
        )
        mapper = RightArmTrajectoryMapper(cls.frames[0].transform)
        cls.targets = np.array([mapper.ik_target(f.transform) for f in cls.frames])

    def test_default_path_is_bundled_and_missing_file_cannot_fall_back_to_22205(self):
        with patch.dict(os.environ, {}, clear=True):
            self.assertEqual(Path(default_right_trajectory_path()), CSV)
        # Test the actual entrypoint function without importing ROS.
        tree = ast.parse((ROOT / "vptele/playback_right_teleop.py").read_text())
        function = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == "default_input_path")
        module = ast.Module(body=[function], type_ignores=[])
        namespace = {"default_right_trajectory_path": default_right_trajectory_path}
        exec(compile(module, "default_input_path", "exec"), namespace)
        with patch.dict(os.environ, {}, clear=True):
            self.assertEqual(Path(namespace["default_input_path"]()), CSV)

    def test_explicit_environment_override_is_preserved(self):
        with patch.dict(os.environ, {"ARM_TELEOP_TRAJECTORY_CSV": "custom.csv"}):
            self.assertEqual(default_right_trajectory_path(), os.path.abspath("custom.csv"))

    def test_initial_state_matches_verified_offset_seed_and_standard_phi(self):
        np.testing.assert_allclose(R30_BALANCED_INITIAL_RIGHT_JOINTS, [
            .0591851471685264, -.4778074189685440, -.2609128124230255,
            1.4799132169734923, -.8665106234837223, -.8948895987437119,
            .1714909921362970], atol=1e-14, rtol=0)
        self.assertAlmostEqual(R30_BALANCED_INITIAL_RIGHT_ARM_ANGLE, -np.pi/4, places=14)
        validate_online_solution(R30_BALANCED_INITIAL_RIGHT_JOINTS)
        self.assertIsNone(self.frames[0].initial_joints)  # Raw format uses these defaults.

    def test_duration_and_sampling(self):
        self.assertAlmostEqual(self.summary.duration, 25.7)
        dt = np.diff([f.timestamp for f in self.frames])
        np.testing.assert_allclose(dt, 1/30, atol=1e-10, rtol=0)

    def test_mapped_first_pose_and_constant_orientation(self):
        np.testing.assert_allclose(self.targets[0, :3], [.3011, -.358, .2282], atol=1e-12)
        expected = np.array([-.0845439162280057, .8477020537874809,
                             -.1008531874967262, -.5138892767951758])
        self.assertTrue(np.isfinite(self.targets).all())
        alignment = np.sum(self.targets[:, 3:] * expected, axis=1)
        np.testing.assert_allclose(np.abs(alignment), 1, atol=1e-12, rtol=0)

    def test_static_segments_and_motion_span(self):
        xyz = self.targets[:, :3]
        np.testing.assert_allclose(xyz[:451] - xyz[0], 0, atol=1e-12)
        np.testing.assert_allclose(xyz[711:] - xyz[711], 0, atol=1e-12)
        np.testing.assert_allclose(xyz[-1], xyz[0], atol=1e-12)
        moving = np.flatnonzero(np.linalg.norm(np.diff(xyz, axis=0), axis=1) > 1e-12) + 1
        self.assertEqual((len(moving), moving[0], moving[-1]), (261, 451, 711))

    def test_radius_and_coordinate_convention(self):
        # Raw VP XZ maps to robot YZ. Do not silently rotate the deployed path.
        xyz = self.targets[:, :3]
        np.testing.assert_allclose(xyz[:, 0], .3011, atol=1e-12)
        radius = np.linalg.norm(xyz[:, 1:] - xyz[0, 1:], axis=1)
        self.assertLessEqual(radius.max(), .030000001)
        self.assertGreater(radius.max(), .0299)


if __name__ == "__main__":
    unittest.main()
