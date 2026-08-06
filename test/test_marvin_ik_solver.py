#!/usr/bin/env python3

import unittest
import tempfile
from pathlib import Path

import numpy as np

from vptele.arm_control.marvin_ik_solver import (
    MarvinIKInitializationError,
    MarvinIKNoSolution,
    MarvinIKSolver,
    preflight_marvin_ik_assets,
)


class _FakeVector:
    def __init__(self, values):
        self.values = list(values)

    def to_list(self):
        return list(self.values)


class _FakeIKParams:
    def __init__(self):
        self.target = None
        self.reference = None
        self.zsp_type = None
        self.zsp_para = None
        self.zsp_angle = None
        self.m_Output_IsOutRange = False
        self.m_Output_IsDeg = [False] * 7
        self.m_Output_IsJntExd = False
        self.m_Output_RetJoint = _FakeVector([0.0] * 7)
        self.m_OutPut_Result_Num = 0

    def set_input_ik_target_tcp(self, values):
        self.target = list(values)

    def set_input_ik_ref_joint(self, values):
        self.reference = list(values)

    def set_input_ik_zsp_type(self, value):
        self.zsp_type = value

    def set_input_ik_zsp_para(self, values):
        self.zsp_para = list(values)

    def set_input_zsp_angle(self, value):
        self.zsp_angle = value


class _FakeVendorModule:
    FX_InvKineSolvePara = _FakeIKParams


class _FakeAPI:
    def __init__(self):
        self.initialized_with = None
        self.fk_joints_deg = None
        self.last_ik_params = None
        self.selected_deg = [10.0, 20.0, 30.0, 40.0, 50.0, 60.0, 70.0]
        self.out_of_range = False
        self.singular = False
        self.joint_limit = False
        self.result_num = 3
        self.left_limits = [
            [-170.0, 170.0, 180.0, 450.0]
            for _ in range(7)
        ]
        self.right_limits = [row[:] for row in self.left_limits]
        self.right_limits[3] = [-145.0, 60.0, 180.0, 900.0]

    def load_config(self, arm_type, config_path):
        self.loaded_config = (arm_type, config_path)
        return {
            "TYPE": [1000, 1001],
            "DH": ["left_dh", "right_dh"],
            "PNVA": [self.left_limits, self.right_limits],
            "BD": ["left_interference", "right_interference"],
        }

    def initial_kine(self, robot_type, dh, pnva, bd):
        self.initialized_with = (robot_type, dh, pnva, bd)
        return True

    def fk(self, joints_deg):
        self.fk_joints_deg = list(joints_deg)
        return [
            [1.0, 0.0, 0.0, 100.0],
            [0.0, 1.0, 0.0, 200.0],
            [0.0, 0.0, 1.0, 300.0],
            [0.0, 0.0, 0.0, 1.0],
        ]

    def ik(self, params):
        self.last_ik_params = params
        params.m_Output_IsOutRange = self.out_of_range
        params.m_Output_IsDeg = [self.singular] + [False] * 6
        params.m_Output_IsJntExd = self.joint_limit
        params.m_Output_RetJoint = _FakeVector(self.selected_deg)
        params.m_OutPut_Result_Num = self.result_num
        return params


def _make_solver(api=None):
    return MarvinIKSolver(
        module_path="unused_fx_kine.py",
        library_path="unused_libKine.so",
        config_path="unused.MvKDCfg",
        arm_type=1,
        zsp_type=0,
        zsp_para=[1.0, 2.0, 3.0, 4.0, 5.0, 6.0],
        zsp_angle_deg=7.5,
        _vendor_module=_FakeVendorModule,
        _api=api or _FakeAPI(),
    )


class MarvinIKSolverTest(unittest.TestCase):
    def test_preflight_reports_missing_vendor_kinematics_library(self):
        module_path = __file__
        with self.assertRaisesRegex(
            MarvinIKInitializationError,
            "libMarvinSDK is not a substitute",
        ):
            preflight_marvin_ik_assets(
                module_path,
                "missing_libKine.so",
                "missing.MvKDCfg",
            )

    def test_preflight_rejects_non_kinematics_binary(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            library = Path(temp_dir) / "libKine.so"
            config = Path(temp_dir) / "robot.MvKDCfg"
            library.write_bytes(b"not a kinematics library")
            config.write_text("TYPE=1017\n", encoding="utf-8")
            with self.assertRaisesRegex(
                MarvinIKInitializationError,
                "missing symbols",
            ):
                preflight_marvin_ik_assets(
                    __file__,
                    str(library),
                    str(config),
                )

    def test_preflight_rejects_windows_robot_control_sdk(self):
        with self.assertRaisesRegex(
            MarvinIKInitializationError,
            "robot-control SDK",
        ):
            preflight_marvin_ik_assets(
                __file__,
                str(Path(__file__).with_name("libMarvinSDK.dll")),
                "missing.MvKDCfg",
            )

    def test_initializes_right_arm_from_vendor_config(self):
        api = _FakeAPI()
        _make_solver(api)
        self.assertEqual(
            api.initialized_with,
            (1001, "right_dh", api.right_limits, "right_interference"),
        )

    def test_forward_converts_radians_to_vendor_degrees(self):
        api = _FakeAPI()
        solver = _make_solver(api)
        joints = np.deg2rad([0.0, 10.0, -20.0, 30.0, -40.0, 50.0, -60.0])

        pose = solver.forward(joints)

        np.testing.assert_allclose(api.fk_joints_deg, np.rad2deg(joints))
        np.testing.assert_allclose(pose[:3, 3], [100.0, 200.0, 300.0])

    def test_forward_rejects_reference_outside_configured_joint_limits(self):
        solver = _make_solver(_FakeAPI())
        joints = np.zeros(7)
        joints[3] = np.deg2rad(90.0)
        with self.assertRaisesRegex(MarvinIKNoSolution, "joint limit"):
            solver.forward(joints)

    def test_solve_uses_reference_selection_and_returns_radians(self):
        api = _FakeAPI()
        solver = _make_solver(api)
        target = np.eye(4)
        target[:3, 3] = [101.0, 202.0, 303.0]
        reference = np.deg2rad([1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0])

        solution = solver.solve(target, reference)

        params = api.last_ik_params
        np.testing.assert_allclose(params.target, target.reshape(-1))
        np.testing.assert_allclose(params.reference, np.rad2deg(reference))
        self.assertEqual(params.zsp_type, 0)
        self.assertEqual(params.zsp_para, [1.0, 2.0, 3.0, 4.0, 5.0, 6.0])
        self.assertEqual(params.zsp_angle, 7.5)
        np.testing.assert_allclose(solution, np.deg2rad(api.selected_deg))

    def test_rejects_unsafe_vendor_results(self):
        failure_cases = (
            ("result_num", 0, "zero solutions"),
            ("out_of_range", True, "outside the reachable workspace"),
            ("singular", True, "singular"),
            ("joint_limit", True, "joint limit"),
        )
        for flag, value, message in failure_cases:
            with self.subTest(flag=flag):
                api = _FakeAPI()
                setattr(api, flag, value)
                solver = _make_solver(api)
                with self.assertRaisesRegex(MarvinIKNoSolution, message):
                    solver.solve(np.eye(4), np.zeros(7))


if __name__ == "__main__":
    unittest.main()
