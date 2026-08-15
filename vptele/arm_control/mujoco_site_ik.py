"""ROS-free Cartesian inverse kinematics built on MuJoCo Jacobians."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, Optional, Sequence

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation


@dataclass(frozen=True)
class IKSolution:
    joints: np.ndarray
    converged: bool
    position_error: float
    orientation_error: float
    iterations: int


class MujocoSiteIK:
    """Damped-least-squares IK using one private ``MjData`` instance.

    MuJoCo exposes forward kinematics and site Jacobians, but no high-level
    whole-arm IK call.  Keeping the iterative solve in a private data object
    makes it deterministic and prevents the teleoperation thread from
    modifying the live simulation state.
    """

    def __init__(
        self,
        model: mujoco.MjModel,
        joint_names: Sequence[str],
        site_name: str,
        *,
        damping: float = 1e-2,
        max_iterations: int = 80,
        max_joint_step: float = 0.12,
        position_tolerance: float = 5e-5,
        orientation_tolerance: float = 2e-3,
        orientation_weight: float = 0.35,
        gravity_compensation: bool = True,
    ) -> None:
        self.model = model
        self.joint_names = tuple(joint_names)
        self.site_name = str(site_name)
        self.damping = float(damping)
        self.max_iterations = int(max_iterations)
        self.max_joint_step = float(max_joint_step)
        self.position_tolerance = float(position_tolerance)
        self.orientation_tolerance = float(orientation_tolerance)
        self.orientation_weight = float(orientation_weight)
        self.gravity_compensation = bool(gravity_compensation)

        self.site_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_SITE, self.site_name
        )
        if self.site_id < 0:
            raise ValueError(f"MuJoCo site not found: {self.site_name}")

        joint_ids = []
        for name in self.joint_names:
            joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if joint_id < 0:
                raise ValueError(f"MuJoCo joint not found: {name}")
            if model.jnt_type[joint_id] not in (
                mujoco.mjtJoint.mjJNT_HINGE,
                mujoco.mjtJoint.mjJNT_SLIDE,
            ):
                raise ValueError(f"IK joint must have one DoF: {name}")
            joint_ids.append(joint_id)

        self.joint_ids = np.asarray(joint_ids, dtype=np.int32)
        self.qpos_addresses = model.jnt_qposadr[self.joint_ids].astype(np.int32)
        self.dof_addresses = model.jnt_dofadr[self.joint_ids].astype(np.int32)
        self._limited = model.jnt_limited[self.joint_ids].astype(bool)
        self._ranges = model.jnt_range[self.joint_ids].copy()
        self.data = mujoco.MjData(model)

    def solve(
        self,
        seed_joints: Iterable[float],
        target_position: Iterable[float],
        target_rotation: Optional[np.ndarray] = None,
        *,
        base_qpos: Optional[np.ndarray] = None,
    ) -> IKSolution:
        q = np.asarray(tuple(seed_joints), dtype=float)
        target_position = np.asarray(target_position, dtype=float).reshape(3)
        if q.shape != (len(self.joint_names),):
            raise ValueError("seed_joints length does not match IK joint_names")
        if not np.all(np.isfinite(q)) or not np.all(np.isfinite(target_position)):
            raise ValueError("IK inputs must be finite")

        if target_rotation is not None:
            target_rotation = np.asarray(target_rotation, dtype=float).reshape(3, 3)
            if not np.all(np.isfinite(target_rotation)):
                raise ValueError("target_rotation must be finite")

        if base_qpos is not None:
            base_qpos = np.asarray(base_qpos, dtype=float)
            if base_qpos.shape != self.data.qpos.shape:
                raise ValueError("base_qpos shape does not match model qpos")
            self.data.qpos[:] = base_qpos

        position_error = float("inf")
        orientation_error = 0.0
        converged = False
        iterations = 0

        for iterations in range(1, self.max_iterations + 1):
            self.data.qpos[self.qpos_addresses] = q
            mujoco.mj_forward(self.model, self.data)

            current_position = self.data.site_xpos[self.site_id].copy()
            err_position = target_position - current_position
            position_error = float(np.linalg.norm(err_position))

            jac_position = np.zeros((3, self.model.nv))
            jac_rotation = np.zeros((3, self.model.nv))
            mujoco.mj_jacSite(
                self.model,
                self.data,
                jac_position,
                jac_rotation if target_rotation is not None else None,
                self.site_id,
            )
            jacobian = jac_position[:, self.dof_addresses]
            error = err_position

            if target_rotation is not None:
                current_rotation = self.data.site_xmat[self.site_id].reshape(3, 3)
                local_delta = current_rotation.T @ target_rotation
                err_rotation = current_rotation @ Rotation.from_matrix(
                    local_delta
                ).as_rotvec()
                orientation_error = float(np.linalg.norm(err_rotation))
                error = np.concatenate(
                    (err_position, self.orientation_weight * err_rotation)
                )
                jacobian = np.vstack(
                    (
                        jacobian,
                        self.orientation_weight
                        * jac_rotation[:, self.dof_addresses],
                    )
                )

            converged = (
                position_error <= self.position_tolerance
                and (
                    target_rotation is None
                    or orientation_error <= self.orientation_tolerance
                )
            )
            if converged:
                break

            regularized = jacobian @ jacobian.T
            regularized.flat[:: regularized.shape[0] + 1] += self.damping**2
            try:
                delta = jacobian.T @ np.linalg.solve(regularized, error)
            except np.linalg.LinAlgError:
                delta = np.linalg.lstsq(jacobian, error, rcond=None)[0]

            delta_norm = float(np.linalg.norm(delta))
            if delta_norm > self.max_joint_step:
                delta *= self.max_joint_step / delta_norm
            q += delta
            q[self._limited] = np.clip(
                q[self._limited],
                self._ranges[self._limited, 0],
                self._ranges[self._limited, 1],
            )

        compensated = self._gravity_compensate(q) if self.gravity_compensation else q
        return IKSolution(
            joints=compensated.copy(),
            converged=converged,
            position_error=position_error,
            orientation_error=orientation_error,
            iterations=iterations,
        )

    def _gravity_compensate(self, joints: np.ndarray) -> np.ndarray:
        """Convert desired qpos to position-actuator targets at static load."""
        self.data.qpos[self.qpos_addresses] = joints
        self.data.qvel[:] = 0.0
        self.data.qacc[:] = 0.0
        mujoco.mj_forward(self.model, self.data)
        inverse_force = np.zeros(self.model.nv)
        mujoco.mj_rne(self.model, self.data, 0, inverse_force)

        result = joints.copy()
        for index, (joint_id, dof_address) in enumerate(
            zip(self.joint_ids, self.dof_addresses)
        ):
            actuator_id = self._position_actuator_for_joint(int(joint_id))
            if actuator_id is None:
                continue
            gain = float(self.model.actuator_gainprm[actuator_id, 0])
            if abs(gain) > 1e-9:
                result[index] += float(inverse_force[dof_address]) / gain
        return result

    def _position_actuator_for_joint(self, joint_id: int) -> Optional[int]:
        for actuator_id in range(self.model.nu):
            if (
                self.model.actuator_trntype[actuator_id]
                == mujoco.mjtTrn.mjTRN_JOINT
                and int(self.model.actuator_trnid[actuator_id, 0]) == joint_id
            ):
                return actuator_id
        return None
