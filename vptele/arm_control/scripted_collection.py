#!/usr/bin/env python3
"""
脚本化 peg-in-hole episode 和自动批量采集的纯 Python 运行器。

每次调用启动一条完整的脚本化 peg-in-hole 插入 episode。
本运行器负责 episode 生命周期管理，实际的插入逻辑在
ScriptedPegInsertionController 中。
该模块是自动采集核心，不导入或依赖 ROS。

本运行器流程：
  1. 复位机械臂到初始关节角
  2. 将固定任务目标（孔或 peg）放到中心位置
  3. 启用遥操作指令接收
  4. 等待复位过渡期
  5. 初始化 MuJoCo Jacobian IK 所需的模型缓存
  6. 按配置覆盖调度采样初始 XY 误差
  7. 启动 HDF5 记录
  8. 调用 controller.run_episode() 执行插入
  9. 停止 HDF5 记录
  10. 返回成功/失败及 episode 路径
"""

from __future__ import annotations

import time
import threading
import shutil
import json
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from types import SimpleNamespace
from typing import Any, Dict, List, Optional

import numpy as np

try:
    from vptele.utils.logger import get_logger
    from vptele.utils.batch_collection import BatchProgress
    from vptele.utils.episode_quality import (
        EpisodeQualityDecision,
        evaluate_episode_quality,
    )
except ModuleNotFoundError:  # Catkin's legacy package_dir exposes utils directly.
    from utils.logger import get_logger
    from utils.batch_collection import BatchProgress
    from utils.episode_quality import EpisodeQualityDecision, evaluate_episode_quality

logger = get_logger()


@dataclass
class EpisodeRunOutcome:
    """Internal result shared by manual review and automatic batches."""

    success: bool
    message: str
    episode_path: str = ""
    kept: bool = False
    decision_reasons: List[str] = field(default_factory=list)
    quality_decision: Optional[EpisodeQualityDecision] = None


class ScriptedInsertionRunner:
    """包装 ScriptedPegInsertionController 的 ROS-free episode 运行器。

    设计目的：
      - 脚本控制器本身不依赖 ROS（可脱离 ROS 测试）
      - 本运行器负责 HDF5 记录、机械臂复位和自动质量审核
      - 通过 threading.Lock 保证同一时间只运行一条 episode
    """

    def __init__(
        self,
        robot_controller,       # MuJoCo 机械臂控制器实例 (共享仿真对象)
        config: Dict[str, Any], # 完整配置文件 (config_arm_right_peg.yaml)
        stop_event: Optional[threading.Event] = None,
    ):
        # 保存外部依赖
        self.rc = robot_controller
        self.cfg = config

        # ------------------------------------------------------------------
        # 延迟导入：避免在模块加载时触发 ROS 初始化
        # ------------------------------------------------------------------
        try:
            from vptele.arm_control.scripted_insertion import (
                ScriptedPegInsertionController,
            )
        except ModuleNotFoundError:
            from arm_control.scripted_insertion import ScriptedPegInsertionController

        # 创建插入控制器实例
        # 传入 robot_controller 共享 MuJoCo 仿真对象
        # 自动采集使用 MuJoCo Jacobian IK，不需要 ROS IK 服务。
        self.controller = ScriptedPegInsertionController(
            robot_controller=self.rc,
            ik_service_proxy=None,
            config=self.cfg,
        )

        # ------------------------------------------------------------------
        # 互斥锁：防止同时触发多条 episode
        # ------------------------------------------------------------------
        self._episode_lock = threading.Lock()
        self._running = False   # 标记当前是否有 episode 正在执行
        self.review_mode = str(self.cfg.get("review_mode", "manual")).lower()
        self.target_episodes = int(self.cfg.get("target_episodes", 0))
        configured_max_attempts = int(self.cfg.get("max_attempts", 0))
        self.max_attempts = configured_max_attempts or max(
            self.target_episodes * 5,
            self.target_episodes,
        )
        self.max_consecutive_rejections = int(
            self.cfg.get("max_consecutive_rejections", 10)
        )
        self.reject_action = str(
            self.cfg.get("reject_action", "quarantine")
        ).lower()
        self._batch_stop_event = stop_event or threading.Event()
        self.batch_finished_event = threading.Event()
        self.batch_completed = False
        self.controller.cancel_event = self._batch_stop_event
        self._batch_thread: Optional[threading.Thread] = None
        self._batch_manifest_path: Optional[Path] = None
        self.batch_stats = {"attempted": 0, "kept": 0, "rejected": 0}
        self.batch_progress = BatchProgress(
            target_kept=self.target_episodes,
            max_attempts=self.max_attempts,
            max_consecutive_rejections=self.max_consecutive_rejections,
        )

        # The scripted lifecycle owns recorder stop/review. Disable the
        # controller's legacy auto-stop thread to avoid two concurrent
        # stop_episode() calls for the same HDF5 file.
        self.rc.task_success_auto_stop_recording = False

    def run_manual_collection(self) -> EpisodeRunOutcome:
        """Run the existing console-reviewed workflow without a ROS service."""
        with self._episode_lock:
            if self._running:
                return EpisodeRunOutcome(
                    success=False,
                    message="A scripted episode is already running.",
                )
            self._running = True   # 加锁标记运行中

        try:
            if self.review_mode == "auto":
                return EpisodeRunOutcome(
                    success=False,
                    message=(
                        "Single-run collection is disabled in automatic batch mode."
                    ),
                )
            if bool(self.cfg.get("manual_review_enter_next_episode", True)):
                return self._run_episode_review_loop()

            # ---- 执行单条 episode ----
            return self._run_one_episode()
        except Exception as exc:
            # 未预料的异常：记录完整调用栈
            logger.exception(f"Scripted episode failed: {exc}")
            return EpisodeRunOutcome(
                success=False,
                message=f"Scripted episode error: {exc}",
            )
        finally:
            # 无论成功/失败/异常，都要解除运行标志
            with self._episode_lock:
                self._running = False

    def start_automatic_batch(self) -> bool:
        """Start the configured retained-count batch in a background thread."""
        if self.review_mode != "auto":
            return False
        if self.target_episodes <= 0:
            raise ValueError("target_episodes must be positive in auto mode")
        with self._episode_lock:
            if self._running:
                return False
            if self._batch_thread is not None and self._batch_thread.is_alive():
                return False
        recorder = self.rc.hdf5_recorder
        if recorder is None:
            raise RuntimeError("HDF5 recorder is unavailable")
        manifest_dir = Path(recorder.output_dir) / "batch_manifests"
        manifest_dir.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        self._batch_manifest_path = manifest_dir / f"batch_{stamp}.jsonl"
        self._batch_stop_event.clear()
        self.batch_finished_event.clear()
        self.batch_completed = False
        self.batch_stats = {"attempted": 0, "kept": 0, "rejected": 0}
        self.batch_progress = BatchProgress(
            target_kept=self.target_episodes,
            max_attempts=self.max_attempts,
            max_consecutive_rejections=self.max_consecutive_rejections,
        )
        batch_thread = threading.Thread(
            target=self._run_automatic_batch,
            name="ScriptedAutomaticBatch",
            daemon=True,
        )
        with self._episode_lock:
            if self._running:
                return False
            if self._batch_thread is not None and self._batch_thread.is_alive():
                return False
            self._running = True
            self._batch_thread = batch_thread
        try:
            self._batch_thread.start()
        except Exception:
            with self._episode_lock:
                self._running = False
            raise
        logger.info(
            "AUTO_BATCH started: target_kept=%d max_attempts=%d manifest=%s",
            self.target_episodes,
            self.max_attempts,
            self._batch_manifest_path,
        )
        return True

    def stop_automatic_batch(self, timeout: float = 10.0) -> None:
        self._batch_stop_event.set()
        thread = self._batch_thread
        if (
            thread is not None
            and thread is not threading.current_thread()
            and thread.is_alive()
        ):
            thread.join(timeout=max(0.0, float(timeout)))
            if thread.is_alive():
                logger.warning("Automatic batch worker did not stop within timeout")

    def _run_automatic_batch(self) -> None:
        completed = False
        try:
            while (
                not self._batch_stop_event.is_set()
                and self.batch_progress.can_attempt
            ):
                attempt = self.batch_progress.begin_attempt()
                self.batch_stats = {
                    key: self.batch_progress.to_dict()[key]
                    for key in ("attempted", "kept", "rejected")
                }
                logger.info(
                    "AUTO_BATCH attempt=%d kept=%d/%d rejected=%d",
                    attempt,
                    self.batch_stats["kept"],
                    self.target_episodes,
                    self.batch_stats["rejected"],
                )
                try:
                    outcome = self._run_one_episode()
                except Exception as exc:
                    logger.exception("AUTO_BATCH attempt %d crashed: %s", attempt, exc)
                    outcome = EpisodeRunOutcome(
                        success=False,
                        kept=False,
                        message=f"Unhandled episode error: {exc}",
                        decision_reasons=[f"unhandled_error:{exc}"],
                    )

                self._append_batch_manifest(attempt, outcome)

                self.batch_progress.register(outcome.kept)
                self.batch_stats = {
                    key: self.batch_progress.to_dict()[key]
                    for key in ("attempted", "kept", "rejected")
                }

            completed = self.batch_progress.complete
            self.batch_completed = completed
            logger.info(
                "AUTO_BATCH finished: completed=%s attempted=%d kept=%d rejected=%d",
                completed,
                self.batch_stats["attempted"],
                self.batch_stats["kept"],
                self.batch_stats["rejected"],
            )
            self._append_batch_summary(completed)
        finally:
            with self._episode_lock:
                self._running = False
            self.batch_finished_event.set()

    def _run_episode_review_loop(self) -> EpisodeRunOutcome:
        """Run scripted episodes until the operator quits the review loop."""
        messages = []
        kept_success = True

        while not self._batch_stop_event.is_set():
            outcome = self._run_one_episode()
            messages.append(outcome.message)
            kept_success = kept_success and bool(outcome.success)

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

        return EpisodeRunOutcome(
            success=kept_success,
            message=(
                f"Scripted review loop stopped after {len(messages)} episode(s). "
                + " | ".join(messages[-3:])
            ),
        )

    # ======================================================================
    # episode 主流程
    # ======================================================================

    def _run_one_episode(self) -> EpisodeRunOutcome:
        """执行一条完整的脚本化插入 episode。

        返回 EpisodeRunOutcome，内容包含：
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
        self._reset_episode_signals()

        # ==================================================================
        # 2. 将固定任务目标放到配置的中心位置。
        #    旧场景移动 wall_task；反向场景移动 fixed_peg_fixture。
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
        # 5. 按覆盖调度采样初始 XY 误差
        #
        #    误差参数会在 episode_metadata 中记录，用于后续数据分析
        #    区分不同误差量级对插入成功率/力反馈的影响。
        #
        #    采样后立即获取 error_info，然后传给 run_episode，
        #    保证实际使用的误差和元数据中记录的一致。
        # ==================================================================
        self.controller._init_dofs()  # 缓存 site ID 和 DOF 地址
        self.controller._sample_error()
        sample_in_hole = getattr(
            self.controller,
            "_sample_in_hole_disturbance",
            None,
        )
        if callable(sample_in_hole):
            sample_in_hole()
        # The threshold must be sampled before metadata is captured. Pass the
        # same value into run_episode() so the controller does not resample it
        # after the HDF5 episode context has already been assembled.
        self.controller._sample_wall_threshold()
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
            return EpisodeRunOutcome(
                success=False,
                message="HDF5 recorder is not available.",
                decision_reasons=["recorder_unavailable"],
            )

        episode_path = recorder.start_episode(
            label=label,
            episode_metadata=episode_metadata,
        )
        if episode_path is None:
            return EpisodeRunOutcome(
                success=False,
                message="Failed to start HDF5 episode.",
                decision_reasons=["record_start_failed"],
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
                wall_threshold_n=error_info["scripted_wall_threshold_n"],
            )
        except Exception as exc:
            logger.exception(f"Controller failed: {exc}")
            recorder.stop_episode(status="scripted_error")
            self.rc.accept_teleop_commands = False
            outcome = EpisodeRunOutcome(
                success=False,
                message=f"Controller error: {exc}",
                episode_path=str(recorder.hdf5_path or ""),
                decision_reasons=[f"controller_error:{exc}"],
            )
            return self._finalize_automatic_failure(outcome)
        approach_metrics = getattr(
            self.controller, "get_episode_metrics", lambda: {}
        )()
        recorder.add_event("scripted_episode_metrics", approach_metrics)

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
        task_success = bool(getattr(self.rc, "task_success_triggered", False))
        execution_success = bool(result.success and task_success)
        if execution_success:
            execution_success = self._wait_for_terminal_hold_completion()
        status = "scripted_success" if execution_success else "scripted_failure"
        recorder.add_event(
            "scripted_outcome",
            {
                "outcome": result.outcome,
                "success": execution_success,
                "retry_count": result.retry_count,
                "duration_s": result.duration_s,
            },
        )
        final_path = recorder.stop_episode(status=status)
        keep_episode, decision = self._review_episode(
            final_path,
            SimpleNamespace(
                success=execution_success,
                outcome=result.outcome if result.success else result.outcome,
            ),
        )
        self._prepare_manual_review()

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
            if execution_success
            else f"failed: {result.outcome}"
        )
        review_str = "kept" if keep_episode else "discarded"
        outcome = EpisodeRunOutcome(
            success=execution_success,
            message=(
                f"Scripted episode {outcome_str}; {review_str} after review. "
                f"Path: {final_path}"
            ),
            episode_path=str(final_path or ""),
            kept=keep_episode,
            decision_reasons=list(decision.reasons) if decision else [],
            quality_decision=decision,
        )
        return outcome

    def _run_one_episode_two_stage_replay(
        self,
        error_info: Dict[str, Any],
    ) -> EpisodeRunOutcome:
        """ACT-style two-stage collection.

        Stage 1 runs the scripted expert without HDF5 training recording and
        stores an in-memory trace of actuator commands plus diagnostic force.
        Stage 2 restores the exact initial MuJoCo state, replays the command
        trace, and lets the normal HDF5 recorder produce the training-format
        observations/action datasets. Stage-1 diagnostics are written to a
        separate stage1_trace.hdf5 file for analysis only.
        """
        recorder = self.rc.hdf5_recorder
        if recorder is None:
            return EpisodeRunOutcome(
                success=False,
                message="HDF5 recorder is not available.",
                decision_reasons=["recorder_unavailable"],
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
        approach_metrics = getattr(
            self.controller, "get_episode_metrics", lambda: {}
        )()

        if not bool(stage1_result.success):
            self.rc.accept_teleop_commands = False
            return EpisodeRunOutcome(
                success=False,
                message=(
                    "Stage1 scripted expert failed; no replay HDF5 saved. "
                    f"outcome={stage1_result.outcome}"
                ),
                decision_reasons=[f"stage1_failed:{stage1_result.outcome}"],
            )

        commands = np.asarray(stage1_trace.get("action_command", []), dtype=np.float64)
        if commands.ndim != 2 or commands.shape[0] == 0 or commands.shape[1] < 7:
            self.rc.accept_teleop_commands = False
            return EpisodeRunOutcome(
                success=False,
                message="Stage1 produced an empty/invalid command trace.",
                decision_reasons=["invalid_stage1_command_trace"],
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
            "replay_action_source": "stage1_trace.hdf5:/stage1_trace/action_command",
            "stage1_ft_convention": (
                "ft_wrench=gravity-compensated world-frame wrench; "
                "ft_wrench_raw=raw sensor-frame wrench"
            ),
            **error_info,
            **approach_metrics,
        }

        episode_path = recorder.start_episode(
            label=label,
            episode_metadata=episode_metadata,
        )
        if episode_path is None:
            self.rc.accept_teleop_commands = False
            return EpisodeRunOutcome(
                success=False,
                message="Failed to start HDF5 replay episode.",
                decision_reasons=["record_start_failed"],
            )

        replay_ok = False
        replay_error = ""
        try:
            replay_ok = self._replay_command_trace(commands, replay_hz)
        except Exception as exc:
            replay_error = str(exc)
            logger.exception(f"TWO_STAGE replay failed: {exc}")

        replay_success = bool(getattr(self.rc, "task_success_triggered", False))
        hold_complete = (
            self._wait_for_terminal_hold_completion() if replay_success else False
        )
        execution_success = bool(
            replay_success and replay_ok and not replay_error and hold_complete
        )
        status = (
            "scripted_replay_success"
            if execution_success
            else "scripted_replay_failure"
        )
        recorder.add_event(
            "scripted_two_stage_outcome",
            {
                "stage1_success": bool(stage1_result.success),
                "stage1_outcome": str(stage1_result.outcome),
                "replay_success": replay_success,
                "replay_completed": bool(replay_ok),
                "terminal_hold_completed": bool(hold_complete),
                "replay_error": replay_error,
                "stage1_num_steps": int(commands.shape[0]),
            },
        )
        final_path = recorder.stop_episode(status=status)

        stage1_path = None
        if save_stage1_trace and final_path is not None:
            stage1_path = self._write_stage1_trace_file(
                final_path=final_path,
                stage1_trace=stage1_trace,
                stage1_result=stage1_result,
                replay_success=execution_success,
                replay_hz=replay_hz,
            )
            if stage1_path is not None:
                self._link_stage2_to_stage1_trace(final_path, stage1_path)

        review_result = SimpleNamespace(
            success=execution_success,
            outcome="success" if execution_success else "replay_incomplete_or_failed",
        )
        keep_episode, decision = self._review_episode(final_path, review_result)
        self._prepare_manual_review()

        self.rc.accept_teleop_commands = False
        if self.cfg.get("reset_arm_on_stop", True):
            self.rc.reset_arm_to_initial_pose()

        review_str = "kept" if keep_episode else "discarded"
        outcome = EpisodeRunOutcome(
            success=execution_success,
            message=(
                f"Two-stage replay {'success' if execution_success else 'failed'}; "
                f"{review_str} after review. Stage2: {final_path}; "
                f"Stage1: {stage1_path}"
            ),
            episode_path=str(final_path or ""),
            kept=keep_episode,
            decision_reasons=list(decision.reasons) if decision else [],
            quality_decision=decision,
        )
        return outcome

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
            while (
                not stop_event.is_set()
                and not self._batch_stop_event.is_set()
            ):
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
                wall_threshold_n=error_info["scripted_wall_threshold_n"],
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

        ee_pos = self.rc.get_site_position(self.controller.moving_site_name)
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

        hole_body_name = self.cfg.get(
            "target_body_name", self.cfg.get("hole_body_name", "wall_task")
        )
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

    def _replay_command_trace(
        self,
        commands: np.ndarray,
        replay_hz: float,
    ) -> bool:
        """Replay internal actuator position commands from stage1."""
        period = 1.0 / max(float(replay_hz), 1e-6)
        t_next = time.time()
        n = min(commands.shape[1], 7)
        for i, command in enumerate(commands):
            if self._batch_stop_event.is_set():
                return False
            with self.rc.lock:
                if bool(
                    getattr(self.rc, "task_success_triggered", False)
                    or getattr(self.rc, "terminal_hold_active", False)
                ):
                    recorder = getattr(self.rc, "hdf5_recorder", None)
                    if recorder is not None:
                        recorder.add_event(
                            "scripted_replay_stopped_on_task_success",
                            {"trace_index": int(i)},
                        )
                    logger.info(
                        "TWO_STAGE replay stopped at %d/%d after task success",
                        i,
                        len(commands),
                    )
                    return True
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

    def _write_stage1_trace_file(
        self,
        final_path,
        stage1_trace: Dict[str, np.ndarray],
        stage1_result,
        replay_success: bool,
        replay_hz: float,
    ) -> Optional[Path]:
        """Write non-training stage1 diagnostics to a separate HDF5 file."""
        import h5py

        stage2_path = Path(final_path)
        if not stage2_path.exists():
            logger.warning(f"Stage1 trace not written: {stage2_path} does not exist")
            return None

        stage1_path = stage2_path.with_name("stage1_trace.hdf5")
        with h5py.File(stage1_path, "w") as f:
            g = f.create_group("stage1_trace")
            for key, values in stage1_trace.items():
                g.create_dataset(key, data=np.asarray(values), compression="gzip")

            g.attrs["is_training_observation"] = 0
            g.attrs["action_command_convention"] = "data.ctrl[0:7] sampled during stage1"
            g.attrs["ft_wrench_convention"] = (
                "gravity-compensated world-frame wrench"
            )
            g.attrs["ft_wrench_raw_convention"] = (
                "raw configured FT sensor-frame wrench"
            )
            g.attrs["stage1_success"] = int(bool(stage1_result.success))
            g.attrs["stage1_outcome"] = str(stage1_result.outcome)
            g.attrs["replay_success"] = int(bool(replay_success))
            g.attrs["replay_hz"] = float(replay_hz)
            g.attrs["paired_stage2_file"] = stage2_path.name

            meta = f.require_group("episode_metadata")
            meta.attrs["collection_method"] = "scripted_two_stage_replay"
            meta.attrs["stage1_trace_saved"] = 1
            meta.attrs["stage1_num_steps"] = int(len(stage1_trace.get("timestamps", [])))
            meta.attrs["stage1_success"] = int(bool(stage1_result.success))
            meta.attrs["stage1_outcome"] = str(stage1_result.outcome)
            meta.attrs["replay_success"] = int(bool(replay_success))
            meta.attrs["replay_action_source"] = "stage1_trace/action_command"
            meta.attrs["paired_stage2_file"] = stage2_path.name
            metrics = getattr(
                self.controller, "get_episode_metrics", lambda: {}
            )()
            for key, value in metrics.items():
                if isinstance(value, (bool, int, float, str, np.number)):
                    meta.attrs[key] = value

        self._write_stage1_summary_json(
            summary_path=stage2_path.with_name("stage1_summary.json"),
            stage1_path=stage1_path,
            stage2_path=stage2_path,
            stage1_trace=stage1_trace,
            stage1_result=stage1_result,
            replay_success=replay_success,
            replay_hz=replay_hz,
        )
        logger.info(f"Stage1 trace written to {stage1_path}")
        return stage1_path

    def _write_stage1_summary_json(
        self,
        summary_path: Path,
        stage1_path: Path,
        stage2_path: Path,
        stage1_trace: Dict[str, np.ndarray],
        stage1_result,
        replay_success: bool,
        replay_hz: float,
    ) -> None:
        """Write a small human-previewable JSON summary for stage1 trace."""
        def shape_of(name: str):
            arr = np.asarray(stage1_trace.get(name, []))
            return list(arr.shape)

        ft = np.asarray(stage1_trace.get("ft_wrench", []), dtype=np.float64)
        if ft.ndim == 2 and ft.shape[1] >= 3 and ft.shape[0] > 0:
            force_norm = np.linalg.norm(ft[:, :3], axis=1)
            force_stats = {
                "max": float(np.nanmax(force_norm)),
                "mean": float(np.nanmean(force_norm)),
                "min": float(np.nanmin(force_norm)),
            }
        else:
            force_stats = {"max": None, "mean": None, "min": None}

        ts = np.asarray(stage1_trace.get("timestamps", []), dtype=np.float64)
        if ts.ndim == 2 and ts.shape[0] > 0:
            duration_wall_s = float(ts[-1, 0] - ts[0, 0])
            duration_sim_s = float(ts[-1, 1] - ts[0, 1]) if ts.shape[1] > 1 else None
        else:
            duration_wall_s = None
            duration_sim_s = None

        summary = {
            "stage1_file": str(stage1_path),
            "stage2_file": str(stage2_path),
            "stage1_success": bool(stage1_result.success),
            "stage1_outcome": str(stage1_result.outcome),
            "replay_success": bool(replay_success),
            "replay_hz": float(replay_hz),
            "num_samples": int(len(stage1_trace.get("timestamps", []))),
            "duration_wall_s": duration_wall_s,
            "duration_sim_s": duration_sim_s,
            "datasets": {
                "timestamps": shape_of("timestamps"),
                "action_command": shape_of("action_command"),
                "joint_pos": shape_of("joint_pos"),
                "ee_pose": shape_of("ee_pose"),
                "ft_wrench": shape_of("ft_wrench"),
                "ft_wrench_raw": shape_of("ft_wrench_raw"),
            },
            "force_norm": force_stats,
            "approach_metrics": getattr(
                self.controller, "get_episode_metrics", lambda: {}
            )(),
            # Full controller diagnostics. ``approach_metrics`` is retained
            # as a compatibility alias for existing analysis scripts.
            "episode_metrics": getattr(
                self.controller, "get_episode_metrics", lambda: {}
            )(),
            "notes": {
                "stage1_is_training_observation": False,
                "stage2_episode_hdf5_is_training_data": True,
                "action_command_convention": "data.ctrl[0:7] sampled during stage1",
                "ft_wrench_convention": (
                    "gravity-compensated world-frame wrench"
                ),
            },
        }

        with summary_path.open("w", encoding="utf-8") as f:
            json.dump(summary, f, indent=2, ensure_ascii=False)
        logger.info(f"Stage1 summary written to {summary_path}")

    def _link_stage2_to_stage1_trace(
        self,
        final_path,
        stage1_path: Path,
    ) -> None:
        """Record the separate stage1 file path in the stage2 training HDF5."""
        import h5py

        stage2_path = Path(final_path)
        if not stage2_path.exists():
            return

        with h5py.File(stage2_path, "a") as f:
            meta = f.require_group("episode_metadata")
            meta.attrs["stage1_trace_saved"] = 1
            meta.attrs["stage1_trace_file"] = stage1_path.name
            meta.attrs["stage1_trace_path"] = str(stage1_path)
            meta.attrs["replay_action_source"] = "stage1_trace.hdf5:/stage1_trace/action_command"

    def _review_episode(self, episode_path, result):
        """Choose keep/discard manually or through the deterministic gate."""
        if self.review_mode == "auto":
            recorder = self.rc.hdf5_recorder
            quality_config = dict(self.cfg.get("auto_quality", {}) or {})
            if recorder is not None:
                quality_config.setdefault("require_images", recorder.record_images)
                quality_config.setdefault("expected_state_hz", recorder.state_hz)
                quality_config.setdefault("expected_force_hz", recorder.force_hz)
                quality_config.setdefault("expected_image_hz", recorder.image_hz)
                camera_names = list(recorder.camera_names)
            else:
                camera_names = []
            decision = evaluate_episode_quality(
                episode_path or "",
                execution_success=bool(result.success),
                config=quality_config,
                expected_camera_names=camera_names,
            )
            if decision.accepted:
                logger.info(
                    "AUTO_REVIEW accepted: %s warnings=%s",
                    episode_path,
                    decision.warnings,
                )
                return True, decision

            try:
                disposition_path = self._dispose_rejected_episode(episode_path)
            except Exception as exc:
                disposition_path = f"disposition_failed:{exc}"
                decision.reasons.append(disposition_path)
                self._batch_stop_event.set()
                logger.exception("Failed to isolate rejected episode: %s", exc)
            decision.metrics["disposition_path"] = disposition_path
            logger.warning(
                "AUTO_REVIEW rejected: %s reasons=%s disposition=%s",
                episode_path,
                decision.reasons,
                disposition_path,
            )
            return False, decision

        if not bool(self.cfg.get("manual_review_after_episode", True)):
            return True, None

        if episode_path is None:
            logger.warning("Manual review skipped: no episode path returned.")
            return True, None

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
                return True, None

            if answer in {"y", "yes"}:
                logger.info(f"Scripted episode kept: {hdf5_path}")
                return True, None

            if answer in {"n", "no"}:
                try:
                    if episode_dir.exists():
                        shutil.rmtree(episode_dir, ignore_errors=True)
                    logger.info(f"Scripted episode discarded: {episode_dir}")
                    return False, None
                except Exception as exc:
                    logger.exception(f"Failed to discard scripted episode: {exc}")
                    return True, None

            print("Please type 'y' to keep or 'n' to discard, then press Enter.")

    def _reset_episode_signals(self) -> None:
        """Clear latched success and safety state before each attempt."""
        reset_task = getattr(self.rc, "_reset_task_success_state_locked", None)
        if reset_task is not None:
            with self.rc.lock:
                reset_task()
        reset_joint = getattr(self.rc, "reset_joint_torque_alarm", None)
        if reset_joint is not None:
            reset_joint()
        reset_ft = getattr(self.rc, "reset_ft_wrench_alarm", None)
        if reset_ft is not None:
            reset_ft()

    def _wait_for_terminal_hold_completion(self) -> bool:
        """Wait until adaptive terminal hold completes or safety-aborts."""
        hold_seconds = max(
            0.0,
            float(getattr(self.rc, "task_success_terminal_hold_time", 0.0)),
        )
        if hold_seconds <= 0.0:
            return True
        deadline = time.monotonic() + hold_seconds + 5.0
        while time.monotonic() < deadline:
            if self._batch_stop_event.is_set():
                return False
            with self.rc.lock:
                if bool(
                    getattr(self.rc, "terminal_hold_safety_aborted", False)
                ):
                    logger.warning(
                        "Terminal hold aborted: %s",
                        getattr(
                            self.rc,
                            "terminal_hold_completion_reason",
                            "unknown",
                        ),
                    )
                    return False
                if bool(getattr(self.rc, "terminal_hold_stop_started", False)):
                    return True
                start = getattr(self.rc, "terminal_hold_start_time", None)
                sim_now = float(self.rc.data.time)
            if start is not None and sim_now - float(start) >= hold_seconds:
                return True
            time.sleep(0.02)
        logger.warning("Timed out waiting for terminal hold completion")
        return False

    def _dispose_rejected_episode(self, episode_path) -> str:
        if not episode_path:
            return "no_episode"
        hdf5_path = Path(episode_path)
        episode_dir = hdf5_path.parent
        if not episode_dir.exists():
            return "already_missing"
        if self.reject_action == "delete":
            shutil.rmtree(episode_dir)
            return "deleted"

        recorder = self.rc.hdf5_recorder
        # Keep rejected HDF5 files outside the accepted dataset root so simple
        # recursive training-data scanners cannot ingest them accidentally.
        output_root = Path(recorder.output_dir)
        quarantine_root = output_root.parent / f"{output_root.name}_rejected"
        quarantine_root.mkdir(parents=True, exist_ok=True)
        destination = quarantine_root / episode_dir.name
        suffix = 1
        while destination.exists():
            destination = quarantine_root / f"{episode_dir.name}_{suffix:03d}"
            suffix += 1
        shutil.move(str(episode_dir), str(destination))
        return str(destination)

    def _finalize_automatic_failure(
        self,
        outcome: EpisodeRunOutcome,
    ) -> EpisodeRunOutcome:
        if self.review_mode != "auto" or not outcome.episode_path:
            return outcome
        quality_config = dict(self.cfg.get("auto_quality", {}) or {})
        quality_config.setdefault("require_images", False)
        decision = evaluate_episode_quality(
            outcome.episode_path,
            execution_success=False,
            config=quality_config,
        )
        try:
            disposition = self._dispose_rejected_episode(outcome.episode_path)
        except Exception as exc:
            disposition = f"disposition_failed:{exc}"
            decision.reasons.append(disposition)
            self._batch_stop_event.set()
            logger.exception("Failed to isolate failed episode: %s", exc)
        decision.metrics["disposition_path"] = disposition
        outcome.decision_reasons = list(decision.reasons)
        outcome.quality_decision = decision
        outcome.kept = False
        return outcome

    def _append_batch_manifest(
        self,
        attempt: int,
        outcome: EpisodeRunOutcome,
    ) -> None:
        if self._batch_manifest_path is None:
            return
        scripted_context = {}
        for method_name in ("get_last_error_info", "get_episode_metrics"):
            method = getattr(self.controller, method_name, None)
            if callable(method):
                scripted_context.update(method())
        row = {
            "timestamp": datetime.now().isoformat(timespec="seconds"),
            "attempt": int(attempt),
            "target_episodes": self.target_episodes,
            "execution_success": bool(outcome.success),
            "kept": bool(outcome.kept),
            "episode_path": outcome.episode_path,
            "reasons": list(outcome.decision_reasons),
            "quality": (
                outcome.quality_decision.to_dict()
                if outcome.quality_decision is not None
                else None
            ),
            "message": outcome.message,
            "scripted_context": scripted_context,
        }
        try:
            with self._batch_manifest_path.open("a", encoding="utf-8") as stream:
                stream.write(json.dumps(row, ensure_ascii=False) + "\n")
        except OSError as exc:
            logger.exception("Could not append batch manifest: %s", exc)
            self._batch_stop_event.set()

    def _append_batch_summary(self, completed: bool) -> None:
        if self._batch_manifest_path is None:
            return
        row = {
            "record_type": "batch_summary",
            "timestamp": datetime.now().isoformat(timespec="seconds"),
            "completed": bool(completed),
            **self.batch_progress.to_dict(),
        }
        try:
            with self._batch_manifest_path.open("a", encoding="utf-8") as stream:
                stream.write(json.dumps(row, ensure_ascii=False) + "\n")
        except OSError as exc:
            logger.exception("Could not append batch summary: %s", exc)

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
        """Place the fixed task target body at its configured nominal pose.

        New scenes use ``fixed_target_world_pos`` and ``target_body_name``.
        The legacy ``fixed_hole_world_pos`` / ``hole_body_name`` names remain
        supported so existing fixed-hole collection is unchanged.

        技术细节：
          - body_pos 是 MuJoCo model 中的 body 初始位置
          - 修改后需要 mj_forward() 刷新所有派生量（site 位置、传感器等）
          - 操作在 self.rc.lock 保护下进行（仿真线程也持此锁）
        """
        import mujoco

        hole_body_name = self.cfg.get(
            "target_body_name", self.cfg.get("hole_body_name", "wall_task")
        )
        body_id = mujoco.mj_name2id(
            self.rc.model,
            mujoco.mjtObj.mjOBJ_BODY,
            hole_body_name,
        )
        if body_id == -1:
            logger.warning(
                f"Task target body '{hole_body_name}' not found; "
                f"target position unchanged."
            )
            return

        # 从配置读取目标位置（默认与 XML 一致）
        default_pos = self.cfg.get(
            "fixed_target_world_pos",
            self.cfg.get("fixed_hole_world_pos", [-0.250, -0.500, 1.000]),
        )
        # 持锁写入并刷新 forward kinematics
        with self.rc.lock:
            self.rc.model.body_pos[body_id] = default_pos
            import mujoco as mj
            mj.mj_forward(self.rc.model, self.rc.data)
        logger.info(f"Task target set to fixed center: {default_pos}")
