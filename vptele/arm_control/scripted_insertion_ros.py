#!/usr/bin/env python3
"""
ROS 节点：对外暴露 /scripted_insertion/run 服务 (std_srvs/Trigger)。

每次调用启动一条完整的脚本化 peg-in-hole 插入 episode。
本节点负责 episode 生命周期管理，实际的插入逻辑在
ScriptedPegInsertionController 中。

本节点流程：
  1. 复位机械臂到初始关节角
  2. 固定孔位到中心位置
  3. 启用遥操作指令接收
  4. 等待复位过渡期
  5. 标定 IK ↔ World 坐标系（若未缓存则运行，此阶段不录 HDF5）
  6. 随机采样初始 XY 误差
  7. 启动 HDF5 记录
  8. 调用 controller.run_episode() 执行插入
  9. 停止 HDF5 记录
  10. 返回成功/失败及 episode 路径
"""

from __future__ import annotations

import time
import threading
import shutil
from pathlib import Path
from types import SimpleNamespace
from typing import Any, Dict, Optional

import numpy as np

import rospy
from std_srvs.srv import Trigger, TriggerResponse

from utils.logger import get_logger

logger = get_logger()


class ScriptedInsertionROSNode:
    """ROS 外挂节点：包装 ScriptedPegInsertionController，提供 Trigger 服务。

    设计目的：
      - 脚本控制器本身不依赖 ROS（可脱离 ROS 测试）
      - 本节点负责 ROS 服务注册、HDF5 记录、机械臂复位等 ROS 相关操作
      - 通过 threading.Lock 保证同一时间只运行一条 episode
    """

    def __init__(
        self,
        robot_controller,       # MuJoCo 机械臂控制器实例 (共享仿真对象)
        ik_service_proxy,       # ROS IK 服务代理 (调用 /arm_teleop/right_arm_ik_srv)
        config: Dict[str, Any], # 完整配置文件 (config_arm_right_peg.yaml)
    ):
        # 保存外部依赖
        self.rc = robot_controller
        self.cfg = config

        # ------------------------------------------------------------------
        # 延迟导入：避免在模块加载时触发 ROS 初始化
        # ------------------------------------------------------------------
        from arm_control.scripted_insertion import (
            ScriptedPegInsertionController,
        )

        # 创建插入控制器实例
        # 传入 robot_controller 共享 MuJoCo 仿真对象
        # 传入 ik_service_proxy 复用 ROS IK 服务连接
        self.controller = ScriptedPegInsertionController(
            robot_controller=self.rc,
            ik_service_proxy=ik_service_proxy,
            config=self.cfg,
        )

        # ------------------------------------------------------------------
        # 互斥锁：防止同时触发多条 episode
        # ------------------------------------------------------------------
        self._episode_lock = threading.Lock()
        self._running = False   # 标记当前是否有 episode 正在执行

        # ------------------------------------------------------------------
        # 注册 ROS Trigger 服务
        # 外部调用: rosservice call /scripted_insertion/run "{}"
        # ------------------------------------------------------------------
        service_name = self.cfg.get(
            "service_name", "/scripted_insertion/run"
        )
        self._service = rospy.Service(
            service_name, Trigger, self._handle_run
        )
        logger.info(
            f"ScriptedInsertion service ready at {service_name}"
        )

    # ======================================================================
    # ROS 服务回调
    # ======================================================================

    def _handle_run(self, _req) -> TriggerResponse:
        """ROS 服务回调：处理 /scripted_insertion/run 请求。

        _req: 空请求体 (Trigger 不需要参数)

        返回:
          TriggerResponse.success = True/False
          TriggerResponse.message = 结果描述（含 episode 路径）
        """
        # ---- 检查是否已有 episode 在运行 ----
        with self._episode_lock:
            if self._running:
                return TriggerResponse(
                    success=False,
                    message="A scripted episode is already running.",
                )
            self._running = True   # 加锁标记运行中

        try:
            if bool(self.cfg.get("manual_review_enter_next_episode", True)):
                return self._run_episode_review_loop()

            # ---- 执行单条 episode ----
            return self._run_one_episode()
        except Exception as exc:
            # 未预料的异常：记录完整调用栈
            logger.exception(f"Scripted episode failed: {exc}")
            return TriggerResponse(
                success=False,
                message=f"Scripted episode error: {exc}",
            )
        finally:
            # 无论成功/失败/异常，都要解除运行标志
            with self._episode_lock:
                self._running = False

    def _run_episode_review_loop(self) -> TriggerResponse:
        """Run scripted episodes until the operator quits the review loop."""
        messages = []
        kept_success = True

        while not rospy.is_shutdown():
            response = self._run_one_episode()
            messages.append(response.message)
            kept_success = kept_success and bool(response.success)

            try:
                answer = input(
                    "\n"
                    "[Scripted Review] Press Enter to collect the next episode, "
                    "or type q then Enter to stop: "
                ).strip().lower()
            except EOFError:
                logger.warning(
                    "Review-loop stdin unavailable; stopping after one episode."
                )
                break

            if answer in {"q", "quit", "exit", "n", "no"}:
                break

        return TriggerResponse(
            success=kept_success,
            message=(
                f"Scripted review loop stopped after {len(messages)} episode(s). "
                + " | ".join(messages[-3:])
            ),
        )

    # ======================================================================
    # episode 主流程
    # ======================================================================

    def _run_one_episode(self) -> TriggerResponse:
        """执行一条完整的脚本化插入 episode。

        返回 TriggerResponse，内容包含：
          - success: 插入是否成功
          - message: 结果描述（成功时含 episode 路径）
        """
        # ==================================================================
        # 1. 复位机械臂到初始关节角
        #    reset_arm_to_initial_pose() 使用 hard_set_qpos 直接写关节位置
        #    (跳过执行器速度限制，快速复位)
        # ==================================================================
        logger.info("Resetting arm to initial pose...")
        self.rc.reset_arm_to_initial_pose()

        # ==================================================================
        # 2. 固定孔位到中心
        #    将 wall_task body 移到 [-0.250, -0.500, 1.000]
        #    不使用 hole_grid_scheduler 的随机孔位
        # ==================================================================
        self._set_fixed_hole_center()

        # ==================================================================
        # 3. 启用遥操作指令接收
        #    set_arm_positions() 内部检查此标志，为 True 才接受指令
        #    (非录制状态下此标志通常为 False，需要显式启用)
        # ==================================================================
        self.rc.accept_teleop_commands = True

        # ==================================================================
        # 4. 等待复位过渡期
        #    reset_arm_to_initial_pose() 设置了一个 ignore_teleop 时间窗口
        #    (默认 0.5s)，在此期间 set_arm_positions 会被忽略。
        #    等待这段时间过去 + 额外 0.1s 缓冲，确保后续指令生效。
        # ==================================================================
        reset_duration = float(
            self.cfg.get("reset_ignore_teleop_duration", 0.5)
        )
        time.sleep(reset_duration + 0.1)

        # ==================================================================
        # 5. 随机采样初始 XY 误差
        #
        #    误差参数会在 episode_metadata 中记录，用于后续数据分析
        #    区分不同误差量级对插入成功率/力反馈的影响。
        #
        #    采样后立即获取 error_info，然后传给 run_episode，
        #    保证实际使用的误差和元数据中记录的一致。
        # ==================================================================
        self.controller._init_dofs()  # 缓存 site ID 和 DOF 地址
        self.controller._sample_error()
        error_info = self.controller.get_last_error_info()

        if str(self.cfg.get("collection_mode", "direct")) == "two_stage_replay":
            return self._run_one_episode_two_stage_replay(error_info)

        # ==================================================================
        # 7. 启动 HDF5 记录
        #
        #    episode_metadata 包含：
        #      - collection_method: "scripted_closed_loop"
        #        (区分人遥操作 "teleop" 和脚本 "scripted_closed_loop")
        #      - scripted_error_xy_mm: 本次 XY 偏移量 (mm)
        #      - scripted_error_angle_deg: 本次偏移方向 (°)
        #
        #    记录内容（由 MujocoHDF5Recorder 自动采集）：
        #      - observations/ee_pose      30Hz  末端位姿
        #      - observations/joint_pos    30Hz  关节角
        #      - observations/joint_vel    30Hz  关节速度
        #      - observations/joint_torque 30Hz  关节力矩
        #      - observations/ft_wrench    500Hz 力传感器 (重力补偿后)
        #      - observations/ft_wrench_raw 500Hz 力传感器 (原始)
        #      - observations/images/ee_cam 30Hz 末端摄像头 640×480
        #      - observations/images/base_top_cam 30Hz 顶部摄像头 640×480
        #      - action                    30Hz 目标关节角指令
        # ==================================================================
        label = self.cfg.get("episode_label", "scripted")
        episode_metadata = {
            "collection_method": "scripted_waypoint",
            **error_info,
        }

        recorder = self.rc.hdf5_recorder
        if recorder is None:
            return TriggerResponse(
                success=False,
                message="HDF5 recorder is not available.",
            )

        episode_path = recorder.start_episode(
            label=label,
            episode_metadata=episode_metadata,
        )
        if episode_path is None:
            return TriggerResponse(
                success=False,
                message="Failed to start HDF5 episode.",
            )
        logger.info(f"HDF5 episode started: {episode_path}")

        # ==================================================================
        # 8. 执行脚本化插入控制器
        #
        #    controller.run_episode() 内部三个阶段：
        #      APPROACH → ALIGN → INSERT
        #
        #    传入预采样的误差参数，确保误差在采样和实际使用之间一致。
        #
        #    异常处理：
        #      控制器内部异常 → stop_episode("scripted_error") → 返回失败
        # ==================================================================
        try:
            result = self.controller.run_episode(
                error_xy_mm=error_info["scripted_error_xy_mm"],
                error_angle_deg=error_info["scripted_error_angle_deg"],
            )
        except Exception as exc:
            logger.exception(f"Controller failed: {exc}")
            recorder.stop_episode(status="scripted_error")
            self.rc.accept_teleop_commands = False
            return TriggerResponse(
                success=False,
                message=f"Controller error: {exc}",
            )

        # ==================================================================
        # 9. 停止 HDF5 记录
        #
        #    写入 scripted_outcome 事件：
        #      - outcome: "success" / "overload" / "stuck" / "ik" 等
        #      - success: True/False
        #      - retry_count: 重试次数
        #      - duration_s: episode 持续时长
        #
        #    stop_episode 状态：
        #      - "scripted_success" (插入成功)
        #      - "scripted_failure" (插入失败)
        # ==================================================================
        status = "scripted_success" if result.success else "scripted_failure"
        recorder.add_event(
            "scripted_outcome",
            {
                "outcome": result.outcome,
                "success": result.success,
                "retry_count": result.retry_count,
                "duration_s": result.duration_s,
            },
        )
        final_path = recorder.stop_episode(status=status)
        self._prepare_manual_review()
        keep_episode = self._review_episode(final_path, result)

        # ==================================================================
        # 10. 清理：禁用遥操作、复位机械臂
        #
        #    reset_arm_on_stop 配置为 false：
        #      控制器内部已有 _retract_clear() 将 peg 退离墙面，
        #      不再需要额外的 hard_reset 跳变。
        #      下一 episode 开始时 _run_one_episode 开头会执行复位。
        # ==================================================================
        self.rc.accept_teleop_commands = False
        if self.cfg.get("reset_arm_on_stop", True):
            self.rc.reset_arm_to_initial_pose()

        # ==================================================================
        # 11. 构建返回消息
        # ==================================================================
        outcome_str = (
            f"success (retries={result.retry_count}, "
            f"dur={result.duration_s:.1f}s)"
            if result.success
            else f"failed: {result.outcome}"
        )
        review_str = "kept" if keep_episode else "discarded"
        return TriggerResponse(
            success=result.success,
            message=(
                f"Scripted episode {outcome_str}; {review_str} after review. "
                f"Path: {final_path}"
            ),
        )

    def _run_one_episode_two_stage_replay(
        self,
        error_info: Dict[str, Any],
    ) -> TriggerResponse:
        """ACT-style two-stage collection.

        Stage 1 runs the scripted expert without HDF5 training recording and
        stores an in-memory trace of actuator commands plus diagnostic force.
        Stage 2 restores the exact initial MuJoCo state, replays the command
        trace, and lets the normal HDF5 recorder produce the training-format
        observations/action datasets. Stage-1 force is appended under
        /stage1_trace/* for analysis only.
        """
        recorder = self.rc.hdf5_recorder
        if recorder is None:
            return TriggerResponse(
                success=False,
                message="HDF5 recorder is not available.",
            )

        stage1_hz = float(self.cfg.get("stage1_trace_hz", 30.0))
        replay_hz = float(self.cfg.get("replay_hz", stage1_hz))
        save_stage1_trace = bool(self.cfg.get("save_stage1_trace", True))

        snapshot = self._snapshot_replay_initial_state()

        logger.info(
            f"TWO_STAGE: stage1 generating expert trace at {stage1_hz:.1f}Hz"
        )
        stage1_result, stage1_trace = self._run_stage1_trace(
            error_info=error_info,
            trace_hz=stage1_hz,
        )

        if not bool(stage1_result.success):
            self.rc.accept_teleop_commands = False
            return TriggerResponse(
                success=False,
                message=(
                    "Stage1 scripted expert failed; no replay HDF5 saved. "
                    f"outcome={stage1_result.outcome}"
                ),
            )

        commands = np.asarray(stage1_trace.get("action_command", []), dtype=np.float64)
        if commands.ndim != 2 or commands.shape[0] == 0 or commands.shape[1] < 7:
            self.rc.accept_teleop_commands = False
            return TriggerResponse(
                success=False,
                message="Stage1 produced an empty/invalid command trace.",
            )

        logger.info(
            f"TWO_STAGE: restoring initial state and replaying {commands.shape[0]} "
            f"commands at {replay_hz:.1f}Hz"
        )
        self._restore_replay_initial_state(snapshot)
        time.sleep(0.05)
        self.rc.accept_teleop_commands = True

        label = self.cfg.get("episode_label", "scripted")
        episode_metadata = {
            "collection_method": "scripted_two_stage_replay",
            "stage1_trace_saved": int(save_stage1_trace),
            "stage1_trace_hz": stage1_hz,
            "replay_hz": replay_hz,
            "stage1_success": int(bool(stage1_result.success)),
            "stage1_outcome": str(stage1_result.outcome),
            "replay_action_source": "stage1_trace/action_command",
            "stage1_ft_convention": "ft_wrench=world-rotated sensor force; ft_wrench_raw=raw sensor frame",
            **error_info,
        }

        episode_path = recorder.start_episode(
            label=label,
            episode_metadata=episode_metadata,
        )
        if episode_path is None:
            self.rc.accept_teleop_commands = False
            return TriggerResponse(
                success=False,
                message="Failed to start HDF5 replay episode.",
            )

        replay_ok = False
        replay_error = ""
        try:
            replay_ok = self._replay_command_trace(commands, replay_hz)
        except Exception as exc:
            replay_error = str(exc)
            logger.exception(f"TWO_STAGE replay failed: {exc}")

        replay_success = bool(getattr(self.rc, "task_success_triggered", False))
        status = "scripted_replay_success" if replay_success else "scripted_replay_failure"
        recorder.add_event(
            "scripted_two_stage_outcome",
            {
                "stage1_success": bool(stage1_result.success),
                "stage1_outcome": str(stage1_result.outcome),
                "replay_success": replay_success,
                "replay_completed": bool(replay_ok),
                "replay_error": replay_error,
                "stage1_num_steps": int(commands.shape[0]),
            },
        )
        final_path = recorder.stop_episode(status=status)

        if save_stage1_trace and final_path is not None:
            self._write_stage1_trace_to_hdf5(
                final_path=final_path,
                stage1_trace=stage1_trace,
                stage1_result=stage1_result,
                replay_success=replay_success,
                replay_hz=replay_hz,
            )

        self._prepare_manual_review()
        review_result = SimpleNamespace(
            success=replay_success,
            outcome="success" if replay_success else "replay_no_success_trigger",
        )
        keep_episode = self._review_episode(final_path, review_result)

        self.rc.accept_teleop_commands = False
        if self.cfg.get("reset_arm_on_stop", True):
            self.rc.reset_arm_to_initial_pose()

        review_str = "kept" if keep_episode else "discarded"
        return TriggerResponse(
            success=replay_success,
            message=(
                f"Two-stage replay {'success' if replay_success else 'failed'}; "
                f"{review_str} after review. Path: {final_path}"
            ),
        )

    def _run_stage1_trace(self, error_info: Dict[str, Any], trace_hz: float):
        """Run scripted expert while sampling a non-training stage1 trace."""
        stop_event = threading.Event()
        trace = {
            "timestamps": [],
            "action_command": [],
            "joint_pos": [],
            "ee_pose": [],
            "ft_wrench": [],
            "ft_wrench_raw": [],
        }

        def sampler():
            period = 1.0 / max(float(trace_hz), 1e-6)
            t0 = time.time()
            next_t = t0
            while not stop_event.is_set() and not rospy.is_shutdown():
                sample = self._capture_stage1_sample(t0)
                for key, value in sample.items():
                    trace[key].append(value)
                next_t += period
                time.sleep(max(0.0, next_t - time.time()))

        thread = threading.Thread(target=sampler, daemon=True)
        thread.start()
        try:
            result = self.controller.run_episode(
                error_xy_mm=error_info["scripted_error_xy_mm"],
                error_angle_deg=error_info["scripted_error_angle_deg"],
            )
        finally:
            stop_event.set()
            thread.join(timeout=1.0)

        trace_np = {
            key: np.asarray(values, dtype=np.float64)
            for key, values in trace.items()
        }
        logger.info(
            f"TWO_STAGE: stage1 result={result.outcome}, "
            f"success={result.success}, samples={len(trace_np['timestamps'])}"
        )
        return result, trace_np

    def _capture_stage1_sample(self, wall_t0: float) -> Dict[str, np.ndarray]:
        """Capture one stage1 trace sample without touching the HDF5 recorder."""
        with self.rc.lock:
            action_command = np.asarray(self.rc.data.ctrl[:7], dtype=np.float64).copy()
            joint_pos = np.asarray(self.rc.data.qpos[:7], dtype=np.float64).copy()
            t_sim = float(self.rc.data.time)

        ee_pos = self.rc.get_site_position("peg_tip_site")
        ee_pose = np.full(7, np.nan, dtype=np.float64)
        if ee_pos is not None:
            ee_pose[:3] = np.asarray(ee_pos, dtype=np.float64)
            ee_pose[3] = 1.0

        ft_raw = self.rc.get_peg_ft_sensor()
        if ft_raw is None:
            ft_raw_arr = np.full(6, np.nan, dtype=np.float64)
        else:
            ft_raw_arr = np.asarray(ft_raw, dtype=np.float64)

        ft_world = self.controller._ft_world()
        if ft_world is None:
            ft_world_arr = np.full(6, np.nan, dtype=np.float64)
        else:
            ft_world_arr = np.asarray(ft_world, dtype=np.float64)

        return {
            "timestamps": np.asarray([time.time() - wall_t0, t_sim], dtype=np.float64),
            "action_command": action_command,
            "joint_pos": joint_pos,
            "ee_pose": ee_pose,
            "ft_wrench": ft_world_arr,
            "ft_wrench_raw": ft_raw_arr,
        }

    def _snapshot_replay_initial_state(self) -> Dict[str, Any]:
        """Snapshot MuJoCo and controller state before stage1."""
        import mujoco

        hole_body_name = self.cfg.get("hole_body_name", "wall_task")
        body_id = mujoco.mj_name2id(
            self.rc.model,
            mujoco.mjtObj.mjOBJ_BODY,
            hole_body_name,
        )
        with self.rc.lock:
            return {
                "qpos": self.rc.data.qpos.copy(),
                "qvel": self.rc.data.qvel.copy(),
                "ctrl": self.rc.data.ctrl.copy(),
                "target_joints": list(self.rc.target_joints),
                "command_joints": list(self.rc.command_joints),
                "body_id": int(body_id),
                "body_pos": self.rc.model.body_pos[body_id].copy() if body_id >= 0 else None,
            }

    def _restore_replay_initial_state(self, snapshot: Dict[str, Any]) -> None:
        """Restore the exact state used at the beginning of stage1."""
        import mujoco

        with self.rc.lock:
            body_id = int(snapshot.get("body_id", -1))
            body_pos = snapshot.get("body_pos")
            if body_id >= 0 and body_pos is not None:
                self.rc.model.body_pos[body_id] = np.asarray(body_pos, dtype=np.float64)

            self.rc.data.qpos[:] = snapshot["qpos"]
            self.rc.data.qvel[:] = snapshot["qvel"]
            self.rc.data.ctrl[:] = snapshot["ctrl"]
            self.rc.target_joints[:] = list(snapshot["target_joints"])
            self.rc.command_joints[:] = list(snapshot["command_joints"])

            if hasattr(self.rc.data, "qacc"):
                self.rc.data.qacc[:] = 0.0
            if hasattr(self.rc.data, "qacc_warmstart"):
                self.rc.data.qacc_warmstart[:] = 0.0
            mujoco.mj_forward(self.rc.model, self.rc.data)

            reset_fn = getattr(self.rc, "_reset_task_success_state_locked", None)
            if reset_fn is not None:
                reset_fn()

    def _replay_command_trace(self, commands: np.ndarray, replay_hz: float) -> bool:
        """Replay internal actuator position commands from stage1."""
        period = 1.0 / max(float(replay_hz), 1e-6)
        t_next = time.time()
        n = min(commands.shape[1], 7)
        for i, command in enumerate(commands):
            if rospy.is_shutdown():
                return False
            with self.rc.lock:
                cmd = [float(v) for v in command[:n]]
                self.rc.target_joints[:n] = cmd
                self.rc.command_joints[:n] = cmd
                self.rc._apply_actuator_targets(self.rc.command_joints)
            if i % 30 == 0:
                logger.info(f"TWO_STAGE replay: {i}/{len(commands)}")
            t_next += period
            time.sleep(max(0.0, t_next - time.time()))
        time.sleep(float(self.cfg.get("replay_final_settle_s", 0.5)))
        return True

    def _write_stage1_trace_to_hdf5(
        self,
        final_path,
        stage1_trace: Dict[str, np.ndarray],
        stage1_result,
        replay_success: bool,
        replay_hz: float,
    ) -> None:
        """Append non-training stage1 diagnostics to the replay HDF5 file."""
        import h5py

        hdf5_path = Path(final_path)
        if not hdf5_path.exists():
            logger.warning(f"Stage1 trace not written: {hdf5_path} does not exist")
            return

        with h5py.File(hdf5_path, "a") as f:
            if "stage1_trace" in f:
                del f["stage1_trace"]
            g = f.create_group("stage1_trace")
            for key, values in stage1_trace.items():
                g.create_dataset(key, data=np.asarray(values), compression="gzip")

            g.attrs["is_training_observation"] = 0
            g.attrs["action_command_convention"] = "data.ctrl[0:7] sampled during stage1"
            g.attrs["ft_wrench_convention"] = "world-rotated sensor wrench, not HDF5 gravity-compensated"
            g.attrs["ft_wrench_raw_convention"] = "raw peg FT sensor frame"
            g.attrs["stage1_success"] = int(bool(stage1_result.success))
            g.attrs["stage1_outcome"] = str(stage1_result.outcome)
            g.attrs["replay_success"] = int(bool(replay_success))
            g.attrs["replay_hz"] = float(replay_hz)

            meta = f.require_group("episode_metadata")
            meta.attrs["collection_method"] = "scripted_two_stage_replay"
            meta.attrs["stage1_trace_saved"] = 1
            meta.attrs["stage1_num_steps"] = int(len(stage1_trace.get("timestamps", [])))
            meta.attrs["stage1_success"] = int(bool(stage1_result.success))
            meta.attrs["stage1_outcome"] = str(stage1_result.outcome)
            meta.attrs["replay_success"] = int(bool(replay_success))
            meta.attrs["replay_action_source"] = "stage1_trace/action_command"

        logger.info(f"Stage1 trace written to {hdf5_path}")

    def _review_episode(self, episode_path, result) -> bool:
        """Ask the operator whether to keep the completed scripted episode."""
        if not bool(self.cfg.get("manual_review_after_episode", True)):
            return True

        if episode_path is None:
            logger.warning("Manual review skipped: no episode path returned.")
            return True

        hdf5_path = Path(episode_path)
        episode_dir = hdf5_path.parent
        outcome = "success" if result.success else f"failed:{result.outcome}"

        while True:
            try:
                answer = input(
                    "\n"
                    f"[Scripted Review] Episode finished ({outcome}).\n"
                    f"  path: {hdf5_path}\n"
                    "  Keep this episode? [y/n]: "
                ).strip().lower()
            except EOFError:
                logger.warning("Manual review stdin unavailable; keeping episode.")
                return True

            if answer in {"y", "yes"}:
                logger.info(f"Scripted episode kept: {hdf5_path}")
                return True

            if answer in {"n", "no"}:
                try:
                    if episode_dir.exists():
                        shutil.rmtree(episode_dir, ignore_errors=True)
                    logger.info(f"Scripted episode discarded: {episode_dir}")
                    return False
                except Exception as exc:
                    logger.exception(f"Failed to discard scripted episode: {exc}")
                    return True

            print("Please type 'y' to keep or 'n' to discard, then press Enter.")

    def _prepare_manual_review(self) -> None:
        """Stop auto-stop bookkeeping before prompting, without moving the arm."""
        reset_fn = getattr(self.rc, "_reset_task_success_state_locked", None)
        if reset_fn is None:
            return

        try:
            with self.rc.lock:
                reset_fn()
        except Exception as exc:
            logger.warning(f"Could not reset task-success state before review: {exc}")

    # ======================================================================
    # 孔位设定
    # ======================================================================

    def _set_fixed_hole_center(self) -> None:
        """将 wall_task body 固定到中心位置。

        脚本化插入使用固定孔位（不使用 hole_grid_scheduler 的随机网格）。
        孔位由配置文件中的 fixed_hole_world_pos 指定，
        默认为 XML 中的 wall_task 默认位置: [-0.250, -0.500, 1.000]。

        技术细节：
          - body_pos 是 MuJoCo model 中的 body 初始位置
          - 修改后需要 mj_forward() 刷新所有派生量（site 位置、传感器等）
          - 操作在 self.rc.lock 保护下进行（仿真线程也持此锁）
        """
        import mujoco

        hole_body_name = self.cfg.get("hole_body_name", "wall_task")
        body_id = mujoco.mj_name2id(
            self.rc.model,
            mujoco.mjtObj.mjOBJ_BODY,
            hole_body_name,
        )
        if body_id == -1:
            logger.warning(
                f"Hole body '{hole_body_name}' not found; "
                f"hole position unchanged."
            )
            return

        # 从配置读取目标位置（默认与 XML 一致）
        default_pos = self.cfg.get(
            "fixed_hole_world_pos",
            [-0.250, -0.500, 1.000],
        )
        # 持锁写入并刷新 forward kinematics
        with self.rc.lock:
            self.rc.model.body_pos[body_id] = default_pos
            import mujoco as mj
            mj.mj_forward(self.rc.model, self.rc.data)
        logger.info(f"Hole set to fixed center: {default_pos}")
