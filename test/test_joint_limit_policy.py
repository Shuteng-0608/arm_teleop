import ast
from pathlib import Path
import unittest

from core.right_joint_playback import JOINT_LIMITS
from core.right_teleop_playback import validate_online_solution
from core.right_arm_trajectory import R30_BALANCED_INITIAL_RIGHT_JOINTS


class JointLimitPolicyTest(unittest.TestCase):
    def test_limits_match_lower_controller(self):
        self.assertEqual(JOINT_LIMITS, (
            (-1.57079632679, 1.57079632679), (-3.14159265359, 0.0),
            (-3.14159265359, 3.14159265359), (0.0, 2.26),
            (-3.14159265359, 3.14159265359), (-1.22, 1.22),
            (-0.5235987756, 0.5235987756),
        ))

    def test_22205_is_rejected(self):
        legacy_22205 = [0.04655617821291043, 0.06699311968130774,
                        0.42473215492166627, 1.4533704244075891,
                        -0.7389712867175264, -0.9277928319910016,
                        0.11098452044740072]
        with self.assertRaisesRegex(ValueError, "q2=.*outside"):
            validate_online_solution(legacy_22205)

    def test_7790_initialization_is_accepted(self):
        validate_online_solution(R30_BALANCED_INITIAL_RIGHT_JOINTS)

    def test_boundaries_are_accepted(self):
        for side in (0, 1):
            validate_online_solution([limits[side] for limits in JOINT_LIMITS])

    def test_initialization_check_precedes_ros_dispatch(self):
        source = Path(__file__).resolve().parents[1] / "vptele/playback_right_teleop.py"
        tree = ast.parse(source.read_text())
        main = next(node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name == "main")
        calls = {node.func.id: node.lineno for node in ast.walk(main)
                 if isinstance(node, ast.Call) and isinstance(node.func, ast.Name)}
        self.assertLess(calls["validate_online_solution"], calls["run_execute"])
        self.assertLess(calls["validate_online_solution"], calls["run_ik_only"])


if __name__ == "__main__":
    unittest.main()
