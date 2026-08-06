#!/usr/bin/env python3
"""Adapter from the vendor ``ik_lib/fx_kine.py`` API to radians/metres code."""

from __future__ import annotations

import importlib.util
import logging
from pathlib import Path
import re
from types import ModuleType
from typing import Optional, Sequence

import numpy as np


LOGGER = logging.getLogger(__name__)


class MarvinIKError(RuntimeError):
    """Base error for the local Marvin inverse-kinematics backend."""


class MarvinIKInitializationError(MarvinIKError):
    """Raised when the vendor library/config cannot be initialized."""


class MarvinIKNoSolution(MarvinIKError):
    """Raised when the vendor solver does not return a safe selected solution."""


def _as_matrix4(value, name: str) -> np.ndarray:
    matrix = np.asarray(value, dtype=np.float64)
    if matrix.shape != (4, 4) or not np.all(np.isfinite(matrix)):
        raise ValueError(f"{name} must be a finite 4x4 matrix")
    return matrix


def load_fx_kine_module(module_path: str) -> ModuleType:
    """Load the checked-in wrapper by path without relying on catkin packaging."""
    path = Path(module_path).expanduser().resolve()
    if not path.is_file():
        raise MarvinIKInitializationError(
            f"Marvin IK Python wrapper not found: {path}"
        )

    spec = importlib.util.spec_from_file_location("arm_teleop_fx_kine", str(path))
    if spec is None or spec.loader is None:
        raise MarvinIKInitializationError(f"Unable to import Marvin IK wrapper: {path}")
    module = importlib.util.module_from_spec(spec)
    try:
        spec.loader.exec_module(module)
    except Exception as exc:
        raise MarvinIKInitializationError(
            f"Unable to import Marvin IK wrapper {path}: {exc}"
        ) from exc
    return module


def preflight_marvin_ik_assets(
    module_path: str,
    library_path: str,
    config_path: str,
) -> None:
    """Fail before Vision Pro/MuJoCo startup when vendor IK assets are absent."""
    module = Path(module_path).expanduser().resolve()
    library = Path(library_path).expanduser().resolve()
    config = Path(config_path).expanduser().resolve()

    if not module.is_file():
        raise MarvinIKInitializationError(
            f"Marvin IK Python wrapper not found: {module}"
        )
    if library.name.lower() in {"libmarvinsdk.so", "libmarvinsdk.dll"}:
        raise MarvinIKInitializationError(
            f"{library} is the robot-control SDK, not the kinematics ABI. "
            "Configure the library exporting FX_Robot_Kine_IK "
            "(normally libKine.dll on Windows or libKine.so on Linux)."
        )
    if not library.is_file():
        raise MarvinIKInitializationError(
            "Marvin inverse-kinematics library is missing: "
            f"{library}. It must export FX_Robot_Kine_IK; "
            "libMarvinSDK is not a substitute."
        )
    required_symbols = {
        "LOADMvCfg",
        "FX_Robot_Init_Type",
        "FX_Robot_Init_Kine",
        "FX_Robot_Init_Lmt",
        "FX_Robot_Kine_FK",
        "FX_Robot_Kine_IK",
    }
    try:
        symbol_strings = {
            match.decode("ascii")
            for match in re.findall(
                rb"[A-Za-z_][A-Za-z0-9_]{3,}",
                library.read_bytes(),
            )
        }
    except OSError as exc:
        raise MarvinIKInitializationError(
            f"Unable to inspect Marvin kinematics library {library}: {exc}"
        ) from exc
    missing_symbols = sorted(required_symbols - symbol_strings)
    if missing_symbols:
        raise MarvinIKInitializationError(
            f"{library} is not the expected Marvin kinematics ABI; missing "
            "symbols: " + ", ".join(missing_symbols)
        )
    if not config.is_file():
        raise MarvinIKInitializationError(
            f"Marvin .MvKDCfg file is missing: {config}"
        )


class MarvinIKSolver:
    """Initialize and call the vendor Marvin kinematics implementation.

    Public joint values are radians. Vendor joint values and ``ZSP_Angle`` are
    degrees. Vendor pose translations retain the units in ``.MvKDCfg`` (the
    supplied wrapper documents them as millimetres).
    """

    def __init__(
        self,
        *,
        module_path: str,
        library_path: str,
        config_path: str,
        arm_type: int = 1,
        zsp_type: int = 0,
        zsp_para: Optional[Sequence[float]] = None,
        zsp_angle_deg: float = 0.0,
        reject_singular: bool = True,
        reject_joint_limit: bool = True,
        tool_matrix: Optional[Sequence[Sequence[float]]] = None,
        _vendor_module: Optional[ModuleType] = None,
        _api=None,
    ) -> None:
        self.module_path = str(Path(module_path).expanduser().resolve())
        self.library_path = str(Path(library_path).expanduser().resolve())
        self.config_path = str(Path(config_path).expanduser().resolve())
        self.arm_type = int(arm_type)
        self.zsp_type = int(zsp_type)
        self.zsp_para = np.asarray(
            zsp_para if zsp_para is not None else [0.0] * 6,
            dtype=np.float64,
        )
        self.zsp_angle_deg = float(zsp_angle_deg)
        self.reject_singular = bool(reject_singular)
        self.reject_joint_limit = bool(reject_joint_limit)

        if self.arm_type not in (0, 1):
            raise ValueError("arm_type must be 0 (left) or 1 (right)")
        if self.zsp_type not in (0, 1):
            raise ValueError("zsp_type must be 0 or 1")
        if self.zsp_para.shape != (6,) or not np.all(np.isfinite(self.zsp_para)):
            raise ValueError("zsp_para must contain six finite values")
        if not np.isfinite(self.zsp_angle_deg):
            raise ValueError("zsp_angle_deg must be finite")

        if _api is None:
            preflight_marvin_ik_assets(
                self.module_path,
                self.library_path,
                self.config_path,
            )

        self.vendor = _vendor_module or load_fx_kine_module(self.module_path)
        if _api is None:
            library = Path(self.library_path)
            try:
                self.api = self.vendor.Marvin_Kine(library_path=self.library_path)
            except (AttributeError, OSError) as exc:
                raise MarvinIKInitializationError(
                    "Unable to load the Marvin kinematics ABI from "
                    f"{library}: {exc}. libMarvinSDK is a "
                    "robot-control SDK and does not export FX_Robot_Kine_IK."
                ) from exc
        else:
            self.api = _api

        config = Path(self.config_path)

        try:
            config_data = self.api.load_config(self.arm_type, self.config_path)
        except Exception as exc:
            raise MarvinIKInitializationError(
                f"Failed to read Marvin kinematics config {config}: {exc}"
            ) from exc
        if not config_data:
            raise MarvinIKInitializationError(
                f"Vendor load_config failed for {config}"
            )

        try:
            pnva = np.asarray(
                config_data["PNVA"][self.arm_type],
                dtype=np.float64,
            )
        except (KeyError, TypeError, ValueError, IndexError) as exc:
            raise MarvinIKInitializationError(
                f"Invalid PNVA joint-limit data in {config}: {exc}"
            ) from exc
        if pnva.shape != (7, 4) or not np.all(np.isfinite(pnva)):
            raise MarvinIKInitializationError(
                f"Invalid PNVA shape in {config}: expected finite 7x4 values"
            )
        self.joint_lower_rad = np.deg2rad(np.minimum(pnva[:, 0], pnva[:, 1]))
        self.joint_upper_rad = np.deg2rad(np.maximum(pnva[:, 0], pnva[:, 1]))

        try:
            initialized = self.api.initial_kine(
                int(config_data["TYPE"][self.arm_type]),
                config_data["DH"][self.arm_type],
                config_data["PNVA"][self.arm_type],
                config_data["BD"][self.arm_type],
            )
        except Exception as exc:
            raise MarvinIKInitializationError(
                f"Failed to initialize Marvin kinematics: {exc}"
            ) from exc
        if not initialized:
            raise MarvinIKInitializationError("Vendor initial_kine returned false")

        if tool_matrix is not None:
            tool = _as_matrix4(tool_matrix, "tool_matrix")
            try:
                if not self.api.set_tool_kine(tool.tolist()):
                    raise MarvinIKInitializationError(
                        "Vendor set_tool_kine returned false"
                    )
            except MarvinIKInitializationError:
                raise
            except Exception as exc:
                raise MarvinIKInitializationError(
                    f"Failed to configure Marvin tool kinematics: {exc}"
                ) from exc

    def forward(self, joints_rad: Sequence[float]) -> np.ndarray:
        joints = np.asarray(joints_rad, dtype=np.float64)
        if joints.shape != (7,) or not np.all(np.isfinite(joints)):
            raise ValueError("joints_rad must contain seven finite values")
        if np.any(joints < self.joint_lower_rad) or np.any(
            joints > self.joint_upper_rad
        ):
            raise MarvinIKNoSolution("Vendor FK reference violates a joint limit")
        pose = self.api.fk(np.rad2deg(joints).tolist())
        if pose is False or pose is None:
            raise MarvinIKNoSolution("Vendor FK failed")
        return _as_matrix4(pose, "vendor FK result")

    def solve(
        self,
        target_pose: Sequence[Sequence[float]],
        reference_joints_rad: Sequence[float],
    ) -> np.ndarray:
        target = _as_matrix4(target_pose, "target_pose")
        reference = np.asarray(reference_joints_rad, dtype=np.float64)
        if reference.shape != (7,) or not np.all(np.isfinite(reference)):
            raise ValueError("reference_joints_rad must contain seven finite values")

        params = self.vendor.FX_InvKineSolvePara()
        params.set_input_ik_target_tcp(target.reshape(-1).tolist())
        params.set_input_ik_ref_joint(np.rad2deg(reference).tolist())
        params.set_input_ik_zsp_type(self.zsp_type)
        params.set_input_ik_zsp_para(self.zsp_para.tolist())
        params.set_input_zsp_angle(self.zsp_angle_deg)

        try:
            result = self.api.ik(params)
        except Exception as exc:
            raise MarvinIKNoSolution(f"Vendor IK call failed: {exc}") from exc
        if result is False or result is None:
            raise MarvinIKNoSolution("Vendor IK returned no solution")
        if int(result.m_OutPut_Result_Num) <= 0:
            raise MarvinIKNoSolution("Vendor IK reported zero solutions")
        if bool(result.m_Output_IsOutRange):
            raise MarvinIKNoSolution("Target pose is outside the reachable workspace")
        if self.reject_singular and any(bool(value) for value in result.m_Output_IsDeg):
            raise MarvinIKNoSolution("Selected IK solution is singular")
        if self.reject_joint_limit and bool(result.m_Output_IsJntExd):
            raise MarvinIKNoSolution("Selected IK solution violates a joint limit")

        selected_deg = np.asarray(
            result.m_Output_RetJoint.to_list(),
            dtype=np.float64,
        )
        if selected_deg.shape != (7,) or not np.all(np.isfinite(selected_deg)):
            raise MarvinIKNoSolution("Vendor IK returned an invalid selected solution")

        selected_rad = np.deg2rad(selected_deg)
        if np.any(selected_rad < self.joint_lower_rad) or np.any(
            selected_rad > self.joint_upper_rad
        ):
            raise MarvinIKNoSolution(
                "Selected IK solution is outside configured joint limits"
            )

        LOGGER.debug(
            "Marvin IK selected one of %s solutions",
            int(result.m_OutPut_Result_Num),
        )
        return selected_rad
