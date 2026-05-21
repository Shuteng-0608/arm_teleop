#!/usr/bin/env python3
"""
MuJoCo controller for the 7-DoF right-arm peg-tool model.

Contact-oriented version:
- default control mode is actuator position control, not direct qpos overwriting;
- MuJoCo dynamics and contacts are stepped with mj_step();
- target joint commands are rate-limited before being sent to actuators;
- original public methods are preserved where possible:
    set_arm_positions(...)
    set_hand_positions(...)
    get_current_joints(...)
    update_positions(...)
    start_simulation(...)
    disconnect(...)

Important:
    In actuator mode, contacts can stop the peg instead of being overwritten
    by direct qpos assignment. This is the key change for real contact behavior.
"""

from __future__ import annotations

import time
import threading
from typing import List, Optional, Dict, Any

import numpy as np
import mujoco
import mujoco.viewer


class RobotControllerMuJoCoPegTool:
    def __init__(self, model_path: str, config: Optional[Dict[str, Any]] = None):
        self.config = config or {}
        self.model_path = model_path

        # -------------------- Load MuJoCo model --------------------
        try:
            self.model = mujoco.MjModel.from_xml_path(model_path)
            self.data = mujoco.MjData(self.model)
            print(f"成功加载模型: {model_path}")
        except Exception as e:
            print(f"加载模型失败: {e}")
            raise RuntimeError(f"无法加载模型: {model_path}") from e

        self.joint_names = self._get_joint_names()
        print(f"模型包含 {len(self.joint_names)} 个关节: {self.joint_names}")

        default_arm_joints = [f"joint_{i}" for i in range(1, 8)]
        self.arm_joint_names = self.config.get("arm_joints", default_arm_joints)
        self.arm_joint_names = [j for j in self.arm_joint_names if j in self.joint_names]

        if len(self.arm_joint_names) != 7:
            print(
                f"警告: 当前识别到的机械臂关节数量为 {len(self.arm_joint_names)}，"
                f"期望为 7。arm_joint_names={self.arm_joint_names}"
            )

        self.actuator_map = self._create_actuator_map()

        # Default: actuator mode for physical contact.
        self.control_mode = self.config.get("control_mode", "actuator")
        if self.control_mode not in {"actuator", "qpos"}:
            print(f"警告: 未知 control_mode={self.control_mode}，自动切换为 actuator")
            self.control_mode = "actuator"

        # Sign convention kept from your original controller.
        self.arm_sign = self.config.get("arm_sign", [-1, 1, 1, -1, 1, 1, 1])

        # Simulation and visualization timing.
        self.sim_timestep = float(self.model.opt.timestep)
        self.viewer_rate = float(self.config.get("viewer_rate", 60.0))
        self.viewer_period = 1.0 / max(self.viewer_rate, 1.0)
        self.realtime = bool(self.config.get("realtime", True))

        # Target smoothing / safety.
        # In actuator mode, set_arm_positions() only changes target_joints.
        # command_joints is moved toward target_joints with a velocity limit.
        self.max_joint_velocity = float(self.config.get("max_joint_velocity", 1.2))  # rad/s
        self.max_joint_step_qpos = float(self.config.get("max_joint_step_qpos", 0.015))  # rad/frame for qpos debug mode

        self.launch_viewer = bool(self.config.get("launch_viewer", True))
        self.viewer_start_wait = float(self.config.get("viewer_start_wait", 1.0))

        self.running = False
        self.viewer_running = False
        self.lock = threading.RLock()
        self.vis_thread: Optional[threading.Thread] = None

        # State targets.
        self.target_joints = [0.0] * len(self.joint_names)
        self.command_joints = [0.0] * len(self.joint_names)

        # Initial pose.
        initial_arm_joints = self.config.get(
            "initial_arm_joints",
            [-0.046, -0.2, 0.0, 1.6, -1.32, 0.005, 0.005],
        )
        initial_internal = self._convert_arm_command_to_internal(initial_arm_joints)
        self._set_internal_joint_targets(initial_internal, immediate=True)
        self._hard_set_qpos(self.command_joints)
        self._apply_actuator_targets(self.command_joints)
        mujoco.mj_forward(self.model, self.data)

        print("MuJoCo peg-tool 仿真器初始化完成")
        print(f"控制模式: {self.control_mode}")
        print(f"MuJoCo timestep: {self.sim_timestep:.6f} s")
        print(f"max_joint_velocity: {self.max_joint_velocity:.3f} rad/s")

        if self.config.get("auto_start", True):
            print("启动 MuJoCo 仿真线程...")
            self.start_simulation()

    # ------------------------------------------------------------------
    # Model information
    # ------------------------------------------------------------------
    def _get_joint_names(self) -> List[str]:
        joint_names = []
        for i in range(self.model.njnt):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, i)
            if name:
                joint_names.append(name)
        return joint_names

    def _create_actuator_map(self) -> Dict[str, Optional[int]]:
        actuator_map: Dict[str, Optional[int]] = {}
        for joint_name in self.joint_names:
            actuator_name = f"motor_{joint_name}"
            actuator_id = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_name
            )
            if actuator_id != -1:
                actuator_map[joint_name] = actuator_id
                print(f"关节 '{joint_name}' 映射到执行器 ID: {actuator_id}")
            else:
                actuator_map[joint_name] = None
                print(f"警告: 未找到关节 '{joint_name}' 的执行器")
        return actuator_map

    def get_current_joints(self) -> List[float]:
        joint_angles = []
        with self.lock:
            for joint_name in self.joint_names:
                joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
                if joint_id == -1:
                    joint_angles.append(0.0)
                    continue

                qpos_addr = self.model.jnt_qposadr[joint_id]
                if qpos_addr < len(self.data.qpos):
                    joint_angles.append(round(float(self.data.qpos[qpos_addr]), 3))
                else:
                    joint_angles.append(0.0)
        return joint_angles

    # ------------------------------------------------------------------
    # Command conversion and target setting
    # ------------------------------------------------------------------
    def _convert_arm_command_to_internal(self, arm_target_joints: List[float]) -> List[float]:
        if len(arm_target_joints) != 7:
            raise ValueError(f"arm_target_joints 应为 7 个，当前为 {len(arm_target_joints)}")

        return [q * s for q, s in zip(arm_target_joints, self.arm_sign)]

    def _set_internal_joint_targets(self, internal_arm_joints: List[float], immediate: bool = False):
        if len(internal_arm_joints) != 7:
            raise ValueError("internal_arm_joints must have length 7")

        with self.lock:
            if len(self.target_joints) < 7:
                raise RuntimeError(f"当前模型关节数为 {len(self.target_joints)}，少于 7 个")

            self.target_joints[:7] = list(internal_arm_joints)

            if immediate:
                self.command_joints[:7] = list(internal_arm_joints)

    def set_arm_positions(self, arm_target_joints: List[float]):
        """
        Public teleoperation interface.

        Input:
            arm_target_joints: 7 joint angles from IK / teleoperation.

        In actuator mode:
            This function only updates target_joints.
            The simulation thread rate-limits command_joints and sends them to actuators.

        In qpos mode:
            It also updates target_joints, and update_positions() will directly write qpos.
        """
        if len(arm_target_joints) != 7:
            print(f"错误: 机械臂目标关节数({len(arm_target_joints)})应为7个")
            return

        internal = self._convert_arm_command_to_internal(arm_target_joints)
        self._set_internal_joint_targets(internal, immediate=False)

    def set_hand_positions(self, hand_target_joints: List[float]):
        """
        peg-tool model has no hand joints. This no-op keeps upper-level code compatible.
        """
        return

    # ------------------------------------------------------------------
    # Low-level state/control
    # ------------------------------------------------------------------
    def _hard_set_qpos(self, target_joints: List[float]):
        """
        Directly set qpos. Use only for reset/initialization or qpos debug mode.
        """
        if len(target_joints) != len(self.joint_names):
            print(
                f"错误: 目标关节数({len(target_joints)})与模型关节数"
                f"({len(self.joint_names)})不匹配"
            )
            return

        for i, joint_name in enumerate(self.joint_names):
            joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
            if joint_id == -1:
                continue

            qpos_addr = self.model.jnt_qposadr[joint_id]
            dof_addr = self.model.jnt_dofadr[joint_id]

            if qpos_addr < len(self.data.qpos):
                self.data.qpos[qpos_addr] = target_joints[i]
            if dof_addr < len(self.data.qvel):
                self.data.qvel[dof_addr] = 0.0

    def update_positions(self, target_joints: List[float]):
        """
        Kept for compatibility with the original controller.

        In contact/actuator mode you should not call this continuously from outside.
        The normal route is:
            set_arm_positions(...) -> target_joints -> actuator ctrl -> mj_step()

        If control_mode == qpos, this performs rate-limited direct qpos update for debugging.
        """
        if self.control_mode == "actuator":
            with self.lock:
                self.target_joints = list(target_joints)
            return

        with self.lock:
            limited = self._limit_qpos_step_locked(target_joints, self.max_joint_step_qpos)
            self.command_joints = limited
            self._hard_set_qpos(self.command_joints)
            mujoco.mj_forward(self.model, self.data)

    def _limit_qpos_step_locked(self, target_joints: List[float], max_step: float) -> List[float]:
        limited = []
        for current, target in zip(self.command_joints, target_joints):
            delta = target - current
            delta = max(-max_step, min(max_step, delta))
            limited.append(current + delta)
        return limited

    def _step_command_toward_target_locked(self, dt: float):
        max_step = self.max_joint_velocity * dt

        for i in range(min(len(self.command_joints), len(self.target_joints))):
            delta = self.target_joints[i] - self.command_joints[i]
            delta = max(-max_step, min(max_step, delta))
            self.command_joints[i] += delta

    def _apply_actuator_targets(self, target_joints: List[float]):
        for i, joint_name in enumerate(self.joint_names):
            actuator_id = self.actuator_map.get(joint_name)
            if actuator_id is not None and actuator_id >= 0:
                self.data.ctrl[actuator_id] = target_joints[i]

    def _physics_step(self):
        with self.lock:
            self._step_command_toward_target_locked(self.sim_timestep)
            self._apply_actuator_targets(self.command_joints)

        mujoco.mj_step(self.model, self.data)

    # ------------------------------------------------------------------
    # Task/debug helpers
    # ------------------------------------------------------------------
    def get_site_position(self, site_name: str) -> Optional[List[float]]:
        site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, site_name)
        if site_id == -1:
            return None
        with self.lock:
            mujoco.mj_forward(self.model, self.data)
            return self.data.site_xpos[site_id].copy().tolist()

    def get_body_position(self, body_name: str) -> Optional[List[float]]:
        body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, body_name)
        if body_id == -1:
            return None
        with self.lock:
            mujoco.mj_forward(self.model, self.data)
            return self.data.xpos[body_id].copy().tolist()

    def get_sensor_data(self, sensor_name: str) -> Optional[List[float]]:
        sensor_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SENSOR, sensor_name)
        if sensor_id == -1:
            return None

        adr = self.model.sensor_adr[sensor_id]
        dim = self.model.sensor_dim[sensor_id]
        with self.lock:
            return self.data.sensordata[adr:adr + dim].copy().tolist()

    def get_peg_ft_sensor(self) -> Optional[List[float]]:
        """
        Return [Fx, Fy, Fz, Tx, Ty, Tz] from MuJoCo force/torque sensors if present.
        The values are expressed in the ft_sensor_site frame.
        """
        f = self.get_sensor_data("peg_ft_force")
        t = self.get_sensor_data("peg_ft_torque")
        if f is None or t is None:
            return None
        return f + t

    def print_task_contacts(self, max_contacts: int = 10):
        """
        Print contact pairs involving the cylindrical peg.
        Useful for verifying that contact detection is active.
        """
        with self.lock:
            mujoco.mj_forward(self.model, self.data)
            n = min(self.data.ncon, max_contacts)
            for i in range(n):
                c = self.data.contact[i]
                g1 = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, c.geom1)
                g2 = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, c.geom2)
                if g1 == "cylindrical_peg" or g2 == "cylindrical_peg":
                    print(f"[contact {i}] {g1} <-> {g2}, dist={c.dist:.6f}")

    # ------------------------------------------------------------------
    # Simulation / visualization thread
    # ------------------------------------------------------------------
    def visualization_thread(self):
        print("启动可视化/物理仿真线程...")

        # Reset and restore initial pose/ctrl.
        with self.lock:
            q0 = list(self.command_joints)
            mujoco.mj_resetData(self.model, self.data)
            self._hard_set_qpos(q0)
            self._apply_actuator_targets(q0)
            mujoco.mj_forward(self.model, self.data)

        try:
            if self.launch_viewer:
                with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
                    print("可视化已启动，按 ESC 或关闭窗口退出")
                    self.viewer_running = True
                    last_sync = time.perf_counter()

                    while viewer.is_running() and self.running:
                        step_start = time.perf_counter()

                        if self.control_mode == "actuator":
                            self._physics_step()
                        else:
                            with self.lock:
                                self.update_positions(self.target_joints)

                        now = time.perf_counter()
                        if now - last_sync >= self.viewer_period:
                            viewer.sync()
                            last_sync = now

                        if self.realtime:
                            elapsed = time.perf_counter() - step_start
                            sleep_time = self.sim_timestep - elapsed
                            if sleep_time > 0:
                                time.sleep(sleep_time)

                    self.viewer_running = False
                    print("可视化窗口关闭，仿真结束")
            else:
                while self.running:
                    step_start = time.perf_counter()

                    if self.control_mode == "actuator":
                        self._physics_step()
                    else:
                        with self.lock:
                            self.update_positions(self.target_joints)

                    if self.realtime:
                        elapsed = time.perf_counter() - step_start
                        sleep_time = self.sim_timestep - elapsed
                        if sleep_time > 0:
                            time.sleep(sleep_time)

        except Exception as e:
            print(f"可视化/仿真错误: {e}")
            import traceback
            traceback.print_exc()
            self.viewer_running = False

    def start_simulation(self):
        if self.running:
            print("仿真已经在运行")
            return

        self.running = True
        self.vis_thread = threading.Thread(target=self.visualization_thread, daemon=True)
        self.vis_thread.start()

        time.sleep(self.viewer_start_wait)
        print("仿真已启动，按 Ctrl+C 停止主程序...")

    def disconnect(self):
        self.running = False
        if self.vis_thread is not None and self.vis_thread.is_alive():
            self.vis_thread.join(timeout=1.0)
        print("仿真停止")


# Compatibility alias.
# If any older code imports RobotControllerMuJoCo from this file, it will still work.
RobotControllerMuJoCo = RobotControllerMuJoCoPegTool


if __name__ == "__main__":
    config = {
        "control_mode": "actuator",
        "launch_viewer": True,
        "auto_start": True,
        "viewer_rate": 60,
        "realtime": True,
        "max_joint_velocity": 1.2,
        "initial_arm_joints": [-0.046, -0.2, 0.0, 1.6, -1.32, 0.005, 0.005],
    }

    try:
        print("创建仿真器...")
        model_path = "/home/stw/pangu/src/arm_teleop/model/right_arm_peg_tool_wall_contact.xml"
        simulator = RobotControllerMuJoCoPegTool(model_path, config)

        while simulator.running:
            time.sleep(1.0)

        print("仿真完成")

    except KeyboardInterrupt:
        print("收到 Ctrl+C，正在退出...")
        try:
            simulator.disconnect()
        except NameError:
            pass
    except Exception as e:
        print(f"仿真器错误: {e}")
        import traceback
        traceback.print_exc()
