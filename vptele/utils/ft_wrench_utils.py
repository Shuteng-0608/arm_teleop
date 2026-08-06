from __future__ import annotations

from typing import Iterable, List, Tuple

import mujoco
import numpy as np


def sensor_vec(model, data, sensor_name: str, dim: int = 3):
    sensor_id = mujoco.mj_name2id(
        model,
        mujoco.mjtObj.mjOBJ_SENSOR,
        sensor_name,
    )
    if sensor_id == -1:
        return None

    adr = int(model.sensor_adr[sensor_id])
    sensor_dim = int(model.sensor_dim[sensor_id])
    if sensor_dim < dim:
        return None

    return data.sensordata[adr:adr + dim].copy().astype(np.float64)


def raw_ft_wrench(
    model,
    data,
    force_sensor_name: str = "peg_ft_force",
    torque_sensor_name: str = "peg_ft_torque",
):
    force = sensor_vec(model, data, force_sensor_name, 3)
    torque = sensor_vec(model, data, torque_sensor_name, 3)
    if force is None or torque is None:
        return None
    return np.concatenate([force, torque]).astype(np.float64)


def ft_sensor_site_id(
    model,
    force_sensor_name: str = "peg_ft_force",
    torque_sensor_name: str = "peg_ft_torque",
) -> int:
    force_sensor_id = mujoco.mj_name2id(
        model,
        mujoco.mjtObj.mjOBJ_SENSOR,
        force_sensor_name,
    )
    if force_sensor_id != -1:
        return int(model.sensor_objid[force_sensor_id])

    torque_sensor_id = mujoco.mj_name2id(
        model,
        mujoco.mjtObj.mjOBJ_SENSOR,
        torque_sensor_name,
    )
    if torque_sensor_id != -1:
        return int(model.sensor_objid[torque_sensor_id])

    return -1


def ft_sensor_pose_world(data, site_id: int) -> Tuple[np.ndarray, np.ndarray]:
    if site_id == -1:
        return np.full(3, np.nan, dtype=np.float64), np.eye(3, dtype=np.float64)

    sensor_pos_w = data.site_xpos[site_id].copy().astype(np.float64)
    R_ws = data.site_xmat[site_id].copy().reshape(3, 3).astype(np.float64)
    return sensor_pos_w, R_ws


def body_ids(model, body_names: Iterable[str]) -> List[int]:
    ids = []
    for body_name in body_names:
        body_id = mujoco.mj_name2id(
            model,
            mujoco.mjtObj.mjOBJ_BODY,
            str(body_name),
        )
        if body_id != -1:
            ids.append(int(body_id))
    return ids


def tool_mass_and_com_world(model, data, tool_body_ids: Iterable[int]):
    total_mass = 0.0
    weighted_com = np.zeros(3, dtype=np.float64)

    for body_id in tool_body_ids:
        mass = float(model.body_mass[int(body_id)])
        if mass <= 0.0:
            continue

        com_w = data.xipos[int(body_id)].copy().astype(np.float64)
        total_mass += mass
        weighted_com += mass * com_w

    if total_mass <= 1e-12:
        return 0.0, np.full(3, np.nan, dtype=np.float64)

    return total_mass, weighted_com / total_mass


def gravity_wrench_sensor_frame(
    model,
    data,
    ft_site_id: int,
    tool_body_ids: Iterable[int],
    gravity_world,
    sensor_sign: float = -1.0,
):
    if ft_site_id == -1:
        return None

    tool_mass, tool_com_w = tool_mass_and_com_world(model, data, tool_body_ids)
    if tool_mass <= 1e-12 or np.any(np.isnan(tool_com_w)):
        return None

    sensor_pos_w, R_ws = ft_sensor_pose_world(data, ft_site_id)
    if np.any(np.isnan(sensor_pos_w)):
        return None

    R_sw = R_ws.T
    gravity_world = np.asarray(gravity_world, dtype=np.float64).reshape(3)
    force_g_w = tool_mass * gravity_world
    force_g_s = R_sw @ force_g_w
    r_com_s = R_sw @ (tool_com_w - sensor_pos_w)
    torque_g_s = np.cross(r_com_s, force_g_s)
    gravity_wrench = np.concatenate([force_g_s, torque_g_s]).astype(np.float64)
    return float(sensor_sign) * gravity_wrench


def compensated_ft_wrench(
    raw_wrench,
    gravity_wrench,
    compensation_mode: str = "gravity",
):
    raw_wrench = np.asarray(raw_wrench, dtype=np.float64).reshape(6)
    compensation_mode = str(compensation_mode).lower()

    if compensation_mode == "none":
        return raw_wrench

    if compensation_mode == "gravity":
        if gravity_wrench is None:
            return None
        gravity_wrench = np.asarray(gravity_wrench, dtype=np.float64).reshape(6)
        return raw_wrench - gravity_wrench

    return None
