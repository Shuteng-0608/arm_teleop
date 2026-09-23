"""ROS-free regression for the R80 CSV, mapper and actual playback initializer."""
import ast
import json
import os
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

from core.right_arm_trajectory import (
    R80_CLEARANCE_INITIAL_RIGHT_JOINTS,
    R80_CLEARANCE_INITIAL_RIGHT_ARM_ANGLE,
    RightArmTrajectoryMapper,
    default_right_trajectory_path,
)
from core.right_teleop_playback import (
    load_teleop_trajectory, rounded_solver_state, validate_online_solution,
)

ROOT = Path(__file__).resolve().parents[1]
DIRECTORY = ROOT / "experiments/r80_pose_300413"
CSV = DIRECTORY / "R80_pose_300413_engineering.csv"


class R80Pose300413Test(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.pose = json.loads((DIRECTORY / "pose.json").read_text())
        cls.frames, cls.summary = load_teleop_trajectory(
            str(CSV), expected_frame_count=1186,
            expected_file_sha256="40c71bce5f56c634bbc52c1e553b60dcd919a865e980d5cc288277d5c0e79e96",
        )
        mapper = RightArmTrajectoryMapper(cls.frames[0].transform)
        cls.targets = np.array([mapper.ik_target(f.transform) for f in cls.frames])
        cls.tree = ast.parse((ROOT / "vptele/playback_right_teleop.py").read_text())

    def playback_namespace(self):
        # Execute actual production definitions, but never import ROS or run a service.
        definitions = [n for n in self.tree.body if isinstance(n, (ast.FunctionDef, ast.ClassDef))
                       and n.name in ("default_input_path", "OnlineRedundancySolver")]
        namespace = dict(default_right_trajectory_path=default_right_trajectory_path,
                         R80_CLEARANCE_INITIAL_RIGHT_JOINTS=R80_CLEARANCE_INITIAL_RIGHT_JOINTS,
                         R80_CLEARANCE_INITIAL_RIGHT_ARM_ANGLE=R80_CLEARANCE_INITIAL_RIGHT_ARM_ANGLE,
                         rounded_solver_state=rounded_solver_state)
        exec(compile(ast.Module(body=definitions, type_ignores=[]), "playback_definitions", "exec"), namespace)
        return namespace

    def test_default_csv_and_entrypoint_are_paired(self):
        with patch.dict(os.environ, {}, clear=True):
            self.assertEqual(Path(default_right_trajectory_path()), CSV)
            self.assertEqual(Path(self.playback_namespace()["default_input_path"]()), CSV)

    def test_environment_override_is_preserved(self):
        with patch.dict(os.environ, {"ARM_TELEOP_TRAJECTORY_CSV": "custom.csv"}):
            self.assertEqual(default_right_trajectory_path(), os.path.abspath("custom.csv"))

    def test_initialization_matches_verified_pose(self):
        np.testing.assert_allclose(R80_CLEARANCE_INITIAL_RIGHT_JOINTS, self.pose["initial_joints_rad"], atol=1e-14, rtol=0)
        self.assertAlmostEqual(R80_CLEARANCE_INITIAL_RIGHT_ARM_ANGLE, self.pose["initial_standard_arm_angle_rad"], places=14)
        validate_online_solution(R80_CLEARANCE_INITIAL_RIGHT_JOINTS)
        self.assertIsNone(self.frames[0].initial_joints)
        self.assertIsNone(self.frames[0].initial_arm_angle)

    def test_actual_solver_default_and_explicit_history(self):
        args = SimpleNamespace(ik_method="A1_minimum_jv", maximum_step=.03, maximum_velocity=.8)
        cls = self.playback_namespace()["OnlineRedundancySolver"]
        solver = cls(None, args, self.frames[0].initial_joints, self.frames[0].initial_arm_angle)
        np.testing.assert_allclose(solver.initial_joints, self.pose["initial_joints_rad"], atol=1e-14, rtol=0)
        self.assertEqual(solver.previous_request_joints, rounded_solver_state(solver.initial_joints))
        self.assertEqual(solver.previous_arm_angle, R80_CLEARANCE_INITIAL_RIGHT_ARM_ANGLE)
        explicit = cls(None, args, [0, -.2, 0, 1, 0, 0, 0], -.5)
        self.assertEqual(explicit.initial_joints, (0, -.2, 0, 1, 0, 0, 0))
        self.assertEqual(explicit.initial_arm_angle, -.5)

    def test_preflight_fallback_and_movej_share_new_initialization(self):
        main = next(n for n in self.tree.body if isinstance(n, ast.FunctionDef) and n.name == "main")
        names = {n.id for n in ast.walk(main) if isinstance(n, ast.Name)}
        self.assertIn("R80_CLEARANCE_INITIAL_RIGHT_JOINTS", names)
        self.assertNotIn("R30_BALANCED_INITIAL_RIGHT_JOINTS", names)
        execute = next(n for n in self.tree.body if isinstance(n, ast.FunctionDef) and n.name == "run_execute")
        movej = next(n for n in ast.walk(execute) if isinstance(n, ast.Call)
                     and isinstance(n.func, ast.Name) and n.func.id == "call_movej")
        self.assertIsInstance(movej.args[1], ast.Attribute)
        self.assertEqual(movej.args[1].value.id, "solver")
        self.assertEqual(movej.args[1].attr, "initial_joints")

    def test_duration_and_sampling(self):
        self.assertAlmostEqual(self.summary.duration, 39.5)
        self.assertEqual(self.summary.file_sha256, self.pose["csv_sha256"])
        np.testing.assert_allclose(np.diff([f.timestamp for f in self.frames]), 1/30, atol=1e-10, rtol=0)

    def test_first_pose_and_fixed_orientation(self):
        np.testing.assert_allclose(self.targets[0, :3], self.pose["center_m"], atol=1e-12, rtol=0)
        self.assertTrue(np.isfinite(self.targets).all())
        dots = np.sum(self.targets[:, 3:] * np.array(self.pose["target_quaternion_wxyz"]), axis=1)
        np.testing.assert_allclose(np.abs(dots), 1, atol=1e-12, rtol=0)

    def test_holds_and_closed_path(self):
        xyz = self.targets[:, :3]
        np.testing.assert_allclose(xyz[:451] - xyz[0], 0, atol=1e-12)
        np.testing.assert_allclose(xyz[1125:] - xyz[1125], 0, atol=1e-12)
        np.testing.assert_allclose(xyz[-1], xyz[0], atol=1e-12)
        moving = np.flatnonzero(np.linalg.norm(np.diff(xyz, axis=0), axis=1) > 1e-12) + 1
        self.assertEqual((len(moving), moving[0], moving[-1]), (675, 451, 1125))

    def test_radius_and_existing_axis_mapping(self):
        xyz = self.targets[:, :3]
        np.testing.assert_allclose(xyz[:, 0], .3011, atol=1e-12)
        radius = np.linalg.norm(xyz[:, 1:] - xyz[0, 1:], axis=1)
        self.assertLessEqual(radius.max(), .080000001)
        self.assertGreater(radius.max(), .0799)


if __name__ == "__main__":
    unittest.main()
