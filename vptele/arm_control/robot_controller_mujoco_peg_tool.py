#!/usr/bin/env python3
"""
MuJoCo controller for the 7-DoF right-arm peg-tool model.

This version is adapted from the original robot_controller_mujoco.py:
- keeps the original public class/method names where possible;
- removes the dexterous-hand control logic;
- supports the new 7-joint peg-tool model;
- uses a kinematic qpos update loop by default, which is better for the
  current teleoperation visualization stage.
"""

import time
import threading
from typing import List, Optional, Dict, Any

import mujoco
import mujoco.viewer


class RobotControllerMuJoCoPegTool:
    def __init__(self, model_path: str, config: Optional[Dict[str, Any]] = None):
        """
        初始化 MuJoCo 仿真器。

        当前版本面向 peg-tool 模型：
        - 模型中只保留 7 个机械臂关节；
        - 原来的灵巧手关节和手部控制接口被保留为空操作，方便上层代码兼容；
        - 默认采用 qpos 直接更新的运动学可视化模式，适合先接入遥操作。
        """
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

        # -------------------- Joint and actuator info --------------------
        self.joint_names = self._get_joint_names()
        print(f"模型包含 {len(self.joint_names)} 个关节: {self.joint_names}")

        # 对当前 peg-tool 模型，arm_joints 应为 joint_1 ~ joint_7。
        default_arm_joints = [f"joint_{i}" for i in range(1, 8)]
        self.arm_joint_names = self.config.get("arm_joints", default_arm_joints)
        self.arm_joint_names = [j for j in self.arm_joint_names if j in self.joint_names]

        if len(self.arm_joint_names) != 7:
            print(
                f"警告: 当前识别到的机械臂关节数量为 {len(self.arm_joint_names)}，"
                f"期望为 7。arm_joint_names={self.arm_joint_names}"
            )

        # 目标关节角，仍然保留为模型关节数量长度，便于 update_positions() 兼容原逻辑。
        self.target_joints = [0.0] * len(self.joint_names)

        # 预计算执行器映射。当前 qpos 运动学模式下不依赖 actuator，
        # 但保留它便于后续切换到 actuator position control。
        self.actuator_map = self._create_actuator_map()

        # 控制模式：
        #   qpos     : 直接写入 data.qpos，然后 mj_forward，适合当前遥操作可视化；
        #   actuator : 写入 data.ctrl，然后 mj_step，后续做动力学接触时可以尝试。
        self.control_mode = self.config.get("control_mode", "qpos")
        if self.control_mode not in {"qpos", "actuator"}:
            print(f"警告: 未知 control_mode={self.control_mode}，自动切换为 qpos")
            self.control_mode = "qpos"

        # 关节符号修正：保持和你原来的 controller 一致。
        self.arm_sign = self.config.get("arm_sign", [-1, 1, 1, -1, 1, 1, 1])

        # 控制频率和 viewer 更新周期。
        self.control_rate = float(self.config.get("control_rate", 100))
        self.dt = 1.0 / max(self.control_rate, 1.0)

        # 是否启动 viewer。teleoperation 阶段通常需要 True。
        self.launch_viewer = bool(self.config.get("launch_viewer", True))
        self.viewer_start_wait = float(self.config.get("viewer_start_wait", 1.0))

        # 线程同步和控制。
        self.running = False
        self.viewer_running = False
        self.lock = threading.Lock()
        self.vis_thread: Optional[threading.Thread] = None

        print("MuJoCo 仿真器初始化完成")
        print(f"控制模式: {self.control_mode}")

        # 设置初始姿态。默认沿用你之前的初始姿态。
        initial_arm_joints = self.config.get(
            "initial_arm_joints",
            [-0.046, -0.2, 0.0, 1.6, -1.32, 0.005, 0.005],
        )
        self.set_arm_positions(initial_arm_joints)

        # 默认保持原行为：构造 controller 后自动启动仿真线程。
        if self.config.get("auto_start", True):
            print("启动 MuJoCo 仿真线程...")
            self.start_simulation()

    # ------------------------------------------------------------------
    # Model information
    # ------------------------------------------------------------------
    def _get_joint_names(self) -> List[str]:
        """获取模型中所有有名称的关节。"""
        joint_names = []
        for i in range(self.model.njnt):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, i)
            if name:
                joint_names.append(name)
        return joint_names

    def _create_actuator_map(self) -> Dict[str, Optional[int]]:
        """创建关节到执行器的映射：joint_1 -> motor_joint_1。"""
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
        """获取当前模型关节角度，按 self.joint_names 顺序返回。"""
        joint_angles = []
        with self.lock:
            for joint_name in self.joint_names:
                joint_id = mujoco.mj_name2id(
                    self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_name
                )
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
    # Low-level state update
    # ------------------------------------------------------------------
    def update_positions(self, target_joints: List[float]):
        """
        直接设置关节位置。

        这是原 controller 的核心行为，保留不变，但修正为适配 7 关节模型。
        该函数只写 qpos，不计算控制力；适合当前遥操作可视化阶段。
        """
        if len(target_joints) != len(self.joint_names):
            print(
                f"错误: 目标关节数({len(target_joints)})与模型关节数"
                f"({len(self.joint_names)})不匹配"
            )
            return

        with self.lock:
            for i, joint_name in enumerate(self.joint_names):
                joint_id = mujoco.mj_name2id(
                    self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_name
                )
                if joint_id == -1:
                    continue

                qpos_addr = self.model.jnt_qposadr[joint_id]
                dof_addr = self.model.jnt_dofadr[joint_id]

                if qpos_addr < len(self.data.qpos):
                    self.data.qpos[qpos_addr] = target_joints[i]
                if dof_addr < len(self.data.qvel):
                    self.data.qvel[dof_addr] = 0.0

    def _apply_actuator_targets(self, target_joints: List[float]):
        """
        将目标关节角写入 position actuator 的 ctrl。
        后续如果要做动力学接触，可以把 config['control_mode'] 改成 'actuator'。
        """
        if len(target_joints) != len(self.joint_names):
            return

        with self.lock:
            for i, joint_name in enumerate(self.joint_names):
                actuator_id = self.actuator_map.get(joint_name)
                if actuator_id is not None and actuator_id >= 0:
                    self.data.ctrl[actuator_id] = target_joints[i]

    # ------------------------------------------------------------------
    # Public control interfaces used by teleoperation
    # ------------------------------------------------------------------
    def set_arm_positions(self, arm_target_joints: List[float]):
        """
        设置机械臂目标关节位置。

        保持原函数名，方便接入你已有的 ArmTeleopMujoco。
        输入仍然是 7 个机械臂关节角；内部保留原来的符号修正逻辑。
        """
        if len(arm_target_joints) != 7:
            print(f"错误: 机械臂目标关节数({len(arm_target_joints)})应为7个")
            return

        arm_target_joints = [q * s for q, s in zip(arm_target_joints, self.arm_sign)]

        with self.lock:
            if len(self.target_joints) < 7:
                print(f"错误: 当前模型关节数为 {len(self.target_joints)}，少于7个")
                return
            self.target_joints[:7] = arm_target_joints.copy()

        print(f"设置机械臂关节位置: {[f'{x:.3f}' for x in arm_target_joints]}")

    def set_hand_positions(self, hand_target_joints: List[float]):
        """
        peg-tool 模型没有手部关节。

        保留该接口是为了兼容原来的上层调用，避免 HandTeleop 或其他模块误调用时报错。
        当前函数不执行任何动作。
        """
        return

    # ------------------------------------------------------------------
    # Useful task/debug helpers
    # ------------------------------------------------------------------
    def get_site_position(self, site_name: str) -> Optional[List[float]]:
        """返回 site 在世界坐标系下的位置，例如 peg_tip_site。"""
        site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, site_name)
        if site_id == -1:
            return None
        with self.lock:
            return self.data.site_xpos[site_id].copy().tolist()

    def get_body_position(self, body_name: str) -> Optional[List[float]]:
        """返回 body 在世界坐标系下的位置。"""
        body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, body_name)
        if body_id == -1:
            return None
        with self.lock:
            return self.data.xpos[body_id].copy().tolist()

    # ------------------------------------------------------------------
    # Simulation / visualization thread
    # ------------------------------------------------------------------
    def visualization_thread(self):
        """可视化/仿真线程函数。"""
        print("启动可视化仿真线程...")

        mujoco.mj_resetData(self.model, self.data)

        # 将初始 target_joints 写入 qpos，避免 reset 后回到零位。
        self.update_positions(self.target_joints)
        mujoco.mj_forward(self.model, self.data)

        try:
            if self.launch_viewer:
                with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
                    print("可视化已启动，按 ESC 或关闭窗口退出")
                    self.viewer_running = True

                    while viewer.is_running() and self.running:
                        if self.control_mode == "qpos":
                            # 运动学遥操作显示：每帧直接把目标关节写入 qpos。
                            self.update_positions(self.target_joints)
                            mujoco.mj_forward(self.model, self.data)
                        else:
                            # 动力学 position actuator 模式。
                            self._apply_actuator_targets(self.target_joints)
                            mujoco.mj_step(self.model, self.data)

                        viewer.sync()
                        time.sleep(self.dt)

                    self.viewer_running = False
                    print("可视化窗口关闭，仿真结束")
            else:
                # 无 viewer 后台模式。
                while self.running:
                    if self.control_mode == "qpos":
                        self.update_positions(self.target_joints)
                        mujoco.mj_forward(self.model, self.data)
                    else:
                        self._apply_actuator_targets(self.target_joints)
                        mujoco.mj_step(self.model, self.data)
                    time.sleep(self.dt)

        except Exception as e:
            print(f"可视化仿真错误: {e}")
            import traceback

            traceback.print_exc()
            self.viewer_running = False

    def start_simulation(self):
        """启动仿真线程。"""
        if self.running:
            print("仿真已经在运行")
            return

        self.running = True
        self.vis_thread = threading.Thread(target=self.visualization_thread, daemon=True)
        self.vis_thread.start()

        time.sleep(self.viewer_start_wait)
        print("仿真已启动，按 Ctrl+C 停止主程序...")

    def disconnect(self):
        """停止仿真。"""
        self.running = False
        print("仿真停止")


if __name__ == "__main__":
    config = {
        "control_rate": 100,
        "control_mode": "qpos",          # 当前推荐：qpos
        "launch_viewer": True,
        "auto_start": True,
        "initial_arm_joints": [-0.046, -0.2, 0.0, 1.6, -1.32, 0.005, 0.005],
    }

    try:
        print("创建仿真器...")
        model_path = "/home/stw/pangu/src/arm_teleop/model/right_arm_peg_tool.xml"
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
