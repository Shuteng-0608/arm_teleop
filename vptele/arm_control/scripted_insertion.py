#!/usr/bin/env python3
"""
Scripted peg-in-hole insertion.

Coordinate system:
  {W}  = MuJoCo world frame
  {IK} = ROS IK base frame (DH parameters)

Transform chain (from calibration):
  p_world = R_w_ik @ p_ik + t_w_ik       (IK → world)
  p_ik    = R_w_ik^T @ (p_world - t_w_ik) (world → IK)

Calibration (once, cached to disk):
  6-direction central-difference probes + SVD → SO(3) rotation matrix

Three phases:  APPROACH → ALIGN → INSERT
"""

from __future__ import annotations

import math, time, mujoco
from dataclasses import dataclass
from enum import Enum
from typing import Any, Callable, Dict, List, Optional

import numpy as np
from scipy.spatial.transform import Rotation as R

try:
    from vptele.utils.logger import get_logger
    from vptele.utils.error_coverage_scheduler import (
        RimContactCoverageScheduler,
    )
    from vptele.utils.in_hole_disturbance_scheduler import (
        InHoleDisturbanceScheduler,
    )
except ModuleNotFoundError:  # Catkin's legacy package_dir exposes utils directly.
    from utils.logger import get_logger
    from utils.error_coverage_scheduler import RimContactCoverageScheduler
    from utils.in_hole_disturbance_scheduler import InHoleDisturbanceScheduler

logger = get_logger()


class Phase(Enum):
    APPROACH = "approach"
    ALIGN = "align"
    INSERT = "insert"
    SUCCESS = "success"
    FAILED = "failed"


@dataclass
class EpisodeResult:
    """脚本 episode 执行结果"""
    success: bool       # 是否成功插入
    outcome: str        # 结果描述 ("success", "ik", "overload", "stuck"...)
    phase: Phase        # 失败时所在的阶段
    retry_count: int    # 重试次数
    duration_s: float   # episode 总耗时 (秒)


class ScriptedPegInsertionController:
    """
    脚本化 peg-in-hole 插入控制器。

    使用 ROS IK 服务进行全局定位（APPROACH、ALIGN 粗调），
    使用 MuJoCo Jacobian IK 进行局部精调（ALIGN 精细阶段、INSERT），
    并加入重力前馈补偿消除执行器稳态误差。
    """

    # 世界 Y 轴负方向 = 插入方向 (peg 沿此方向进入孔)
    INSERT_AXIS_WORLD = np.array([0.0, -1.0, 0.0], dtype=np.float64)
    FORCE_LIMIT = 40.0  # 力过载保护阈值 (N)，超过立即停止

    # ==================================================================
    # 类变量：标定数据在类级别缓存，所有实例共享
    #   _R_w_ik: 3×3 旋转矩阵，IK 坐标 → 世界坐标
    #   _t_w_ik: 3×1 平移向量，IK 坐标 → 世界坐标
    #   _ik_ref_joints: 参考位姿处的关节角 (作为 IK 种子)
    #   _calib_file: 标定缓存文件路径 (磁盘持久化)
    # ==================================================================
    _R_w_ik: Optional[np.ndarray] = None
    _t_w_ik: Optional[np.ndarray] = None
    _ik_ref_joints: Optional[List[float]] = None
    _calib_file: Optional[str] = None

    def __init__(self, robot_controller, ik_service_proxy, config: Dict[str, Any]):
        self.rc = robot_controller           # MuJoCo 机械臂控制器 (共享仿真对象)
        self.ik = ik_service_proxy           # ROS IK 服务代理
        self.cfg = config                    # 配置字典 (config_arm_right_peg.yaml)
        self.rng = np.random.default_rng()
        self.ik_method = str(self.cfg.get("ik_method", "optimal_ref"))
        self.moving_site_name = self.cfg.get("moving_site_name", "peg_tip_site")
        self.target_goal_site_name = self.cfg.get(
            "target_goal_site_name", "hole_goal_site"
        )
        self.target_approach_site_name = self.cfg.get(
            "target_approach_site_name", ""
        )
        self.target_body_name = self.cfg.get(
            "target_body_name", self.cfg.get("hole_body_name", "wall_task")
        )
        self._insert_axis = np.asarray(
            self.cfg.get("insertion_axis_world", self.INSERT_AXIS_WORLD),
            dtype=np.float64,
        ).reshape(3)
        axis_norm = float(np.linalg.norm(self._insert_axis))
        if axis_norm <= 1e-9:
            raise ValueError("insertion_axis_world must be non-zero")
        self._insert_axis /= axis_norm

        # ==================================================================
        # IK 参考位姿：来自配置文件 initial_robot_pose
        # 格式: [x, y, z, roll, pitch, yaw] (IK 坐标系)
        # 这是一个"安全"的位姿，机械臂在此位置远离墙面和关节极限
        # ==================================================================
        rp = self.cfg.get("initial_robot_pose",
                          [0.3011, -0.358, 0.2282, 3.1923, -0.0361, -0.0008])
        self._ik_ref = np.asarray(rp[:3], dtype=np.float64)       # IK 参考位置
        qs = R.from_euler("XYZ", rp[3:6], degrees=False).as_quat()  # 欧拉角 → 四元数
        self._ik_q = np.asarray([qs[3], qs[0], qs[1], qs[2]],    # MuJoCo 格式 [w,x,y,z]
                                dtype=np.float64)

        # ==================================================================
        # arm_sign: 关节角符号约定转换
        # 外部 (ROS IK)  ↔  内部 (MuJoCo qpos)
        # 内部 = 外部 * arm_sign
        # ==================================================================
        self._arm_sign = getattr(self.rc, "arm_sign", [1] * 7)

        # 随机 XY 误差 (每 episode 采样一次)
        self._err_xy = 0.0       # 偏移量 (mm)
        self._err_deg = 0.0      # 偏移方向 (°)
        self._err_w = np.zeros(3)  # 世界坐标偏移向量 [dx, 0, dz]
        self._error_sample_metadata: Dict[str, Any] = {}
        self._error_scheduler = None
        coverage_mode = str(
            self.cfg.get("error_coverage_mode", "random_fixed_radius")
        ).lower()
        if coverage_mode == "stratified_radius_angle":
            radii = self.cfg.get(
                "rim_contact_radii_mm",
                self.cfg.get(
                    "error_magnitudes_mm",
                    [self.cfg.get("scripted_error_radius_mm", 10.0)],
                ),
            )
            self._error_scheduler = RimContactCoverageScheduler(
                radii_mm=radii,
                angle_bins=int(self.cfg.get("rim_contact_angle_bins", 24)),
                order=self.cfg.get("error_coverage_order", "shuffled"),
                seed=int(self.cfg.get("error_coverage_seed", 42)),
                angle_jitter_deg=float(
                    self.cfg.get("rim_contact_angle_jitter_deg", 0.0)
                ),
                start_cycle=int(self.cfg.get("error_coverage_start_cycle", 0)),
                start_index=int(self.cfg.get("error_coverage_start_index", 0)),
            )
        elif coverage_mode != "random_fixed_radius":
            raise ValueError(
                "error_coverage_mode must be random_fixed_radius or "
                "stratified_radius_angle"
            )

        # 参考世界位置：标定后 peg 在 IK 参考位姿处的位置
        self._ref_w: Optional[np.ndarray] = None
        self._ref_xmat: Optional[np.ndarray] = None

        # 每个 episode 开始时会重新采样墙接触力阈值。
        self._wall_threshold = 0.0
        self._episode_metrics: Dict[str, Any] = {}
        self._event_callback: Optional[
            Callable[[str, Dict[str, Any]], None]
        ] = None
        self._in_hole_phase = "inactive"
        self._in_hole_phase_code = 0
        self._expert_action_mask = 1
        self._in_hole_disturbance_metadata: Dict[str, Any] = {}
        self._in_hole_disturbance_sample = None
        self._in_hole_disturbance_scheduler = None
        if bool(self.cfg.get("in_hole_correction_enabled", False)):
            self._in_hole_disturbance_scheduler = InHoleDisturbanceScheduler(
                depth_fractions=self.cfg.get(
                    "in_hole_disturbance_depth_fractions",
                    [0.30, 0.55, 0.80],
                ),
                direction_bins=int(
                    self.cfg.get("in_hole_disturbance_direction_bins", 8)
                ),
                amplitudes_mm=self.cfg.get(
                    "in_hole_disturbance_amplitudes_mm",
                    [3.4, 3.8],
                ),
                order=self.cfg.get(
                    "in_hole_disturbance_coverage_order", "shuffled"
                ),
                seed=int(
                    self.cfg.get("in_hole_disturbance_coverage_seed", 73)
                ),
                start_cycle=int(
                    self.cfg.get("in_hole_disturbance_start_cycle", 0)
                ),
                start_index=int(
                    self.cfg.get("in_hole_disturbance_start_index", 0)
                ),
            )

        # ==================================================================
        # 标定缓存文件路径
        # 存储在 HDF5 数据目录下的 calib_ik_to_world.npz
        # 首次运行后缓存，后续启动直接加载，跳过 ~30s 的标定过程
        # ==================================================================
        from pathlib import Path

        config_path = Path(str(self.cfg.get("config_path", ""))).expanduser()
        output_dir = Path(
            str(
                self.cfg.get(
                    "hdf5_record_dir",
                    "../../data/hole_random_60mm_hmj",
                )
            )
        ).expanduser()
        if not output_dir.is_absolute():
            base_dir = config_path.parent if str(config_path) else Path.cwd()
            output_dir = base_dir / output_dir
        ScriptedPegInsertionController._calib_file = str(
            output_dir.resolve() / "calib_ik_to_world.npz"
        )

        # MuJoCo 对象缓存 (延迟初始化)
        self._site_id: Optional[int] = None    # moving task site's MuJoCo ID
        self._arm_dofs: List[int] = []          # 7 个关节的 DOF 地址

    # ==================================================================
    # Public — 外部接口
    # ==================================================================

    def run_episode(
        self,
        error_xy_mm=None,
        error_angle_deg=None,
        wall_threshold_n=None,
    ) -> EpisodeResult:
        """
        执行一条完整的 peg-in-hole 插入 episode。

        APPROACH(瞄偏撞墙, 力反馈) → 回撤 → waypoint 对准 → waypoint 插入。
        全程使用 MuJoCo Jacobian IK，不依赖 ROS IK 和标定。

        参数:
          error_xy_mm:      XY 偏移量 (mm)，None 则随机采样
          error_angle_deg:  XY 偏移方向 (°)，None 则随机采样
          wall_threshold_n: 预先采样的墙接触力阈值 (N)。None 则在此采样。

        返回:
          EpisodeResult: success=True 表示插入成功
        """
        t0 = time.time()
        self._episode_metrics = {
            "scripted_approach_started_wall_time": float(t0),
            "scripted_contact_detected": 0,
            "scripted_contact_peak_force_n": 0.0,
            "scripted_contact_push_depth_mm": 0.0,
            "scripted_force_control_source": (
                "gravity_compensated_world_wrench"
            ),
            "scripted_in_hole_correction_enabled": int(
                bool(self.cfg.get("in_hole_correction_enabled", False))
            ),
        }
        self._set_in_hole_phase("inactive")
        if (
            getattr(self, "_in_hole_disturbance_scheduler", None) is not None
            and getattr(self, "_in_hole_disturbance_sample", None) is None
        ):
            self._sample_in_hole_disturbance()
        timeout_s = max(1.0, float(self.cfg.get("episode_timeout_s", 180.0)))
        self._episode_deadline = time.monotonic() + timeout_s
        if wall_threshold_n is None:
            self._sample_wall_threshold()
        else:
            threshold = float(wall_threshold_n)
            if not np.isfinite(threshold) or threshold <= 0.0:
                raise ValueError(
                    "wall_threshold_n must be a positive finite number"
                )
            self._wall_threshold = threshold

        # ---- 设置初始 XY 误差 ----
        if error_xy_mm is not None:
            # 使用预先采样的误差 (ROS 节点已采样)
            self._err_xy = float(error_xy_mm)
            self._err_deg = float(error_angle_deg)
            a = math.radians(self._err_deg)
            m = self._err_xy / 1000
            self._err_w = np.array([m * math.cos(a), 0, m * math.sin(a)])
        else:
            self._sample_error()

        # ---- 惰性初始化 MuJoCo ID 缓存 ----
        self._init_dofs()

        # ---- 记录当前 peg 世界位置和完整姿态 (episode 内固定) ----
        self._ref_w = self._peg_w()
        self._ref_xmat = self._peg_xmat()

        # ---- 阶段 1: APPROACH — 瞄偏接近墙面，撞墙即停 ----
        r = self._approach()
        if r: return r
        # TODO
        # ---- APPROACH 结束后，先冻结接触状态，再沿世界 +Y 平滑回撤 ----
        retract_mm = float(self.rng.uniform(3.0, 8.0))
        p = self._peg_w()
        rt = p.copy()
        rt[1] += retract_mm / 1000.0

        self._freeze_current_arm("wall contact")
        time.sleep(float(self.cfg.get("post_contact_settle_s", 0.15)))

        if not self._smooth_move_ee(
            target_w=rt,
            total_steps=int(self.cfg.get("smooth_retract_steps", 18)),
            step_wait_s=float(self.cfg.get("smooth_retract_step_wait_s", 0.04)),
            constrain_ori=True,
            phase=Phase.ALIGN,
            log_prefix="SMOOTH_RETRACT",
        ):
            return EpisodeResult(False, "ik", Phase.ALIGN, 0, 0.0)

        self._wait_force_release(
            threshold_n=float(self.cfg.get("post_retract_force_release_n", 6.0)),
            timeout_s=float(self.cfg.get("post_retract_force_release_timeout_s", 1.0)),
        )
        logger.info(f"APPROACH smoothly retracted {retract_mm:.1f}mm")
        # ---- 阶段 2: waypoint 对准 + 插入（已知孔位置，直进直出） ----
        start_w = self._peg_w().copy()
        hole_g = self._hole_goal().copy()
        logger.info(f"WAYPOINT: from {[round(v,4) for v in start_w]} → hole {[round(v,4) for v in hole_g]}")

        waypoints_w = self._build_waypoints(start_w, hole_g)
        if not waypoints_w:
            return EpisodeResult(False, "waypoint", Phase.INSERT, 0, 0.0)

        r = self._execute_waypoints(waypoints_w)
        if r is not None:
            return r

        # Completing the waypoint list is not itself proof of insertion. In
        # direct recording mode the debounced task-success state is preferred;
        # stage-1 of two-stage replay has no active recorder, so it falls back
        # to the same geometric site-distance threshold.
        task_triggered = bool(getattr(self.rc, "task_success_triggered", False))
        if not task_triggered and not self._task_goal_reached():
            return EpisodeResult(
                False,
                "task_success_not_reached",
                Phase.INSERT,
                0,
                time.time() - t0,
            )

        # ---- 成功：退 peg 后再返回 (避免复位时撞墙) ----
        self._retract_clear()
        return EpisodeResult(True, "success", Phase.SUCCESS, 0, time.time() - t0)

    def _episode_timed_out(self) -> bool:
        cancel_event = getattr(self, "cancel_event", None)
        if cancel_event is not None and cancel_event.is_set():
            return True
        deadline = getattr(self, "_episode_deadline", None)
        return deadline is not None and time.monotonic() >= float(deadline)

    def _task_goal_reached(self) -> bool:
        peg = self._peg_w()
        goal = self._hole_goal()
        threshold = float(getattr(self.rc, "task_success_distance", 0.003))
        return float(np.linalg.norm(peg - goal)) <= threshold

    def get_last_error_info(self):
        """获取最近一次采样的 XY 误差信息 (用于写入 episode_metadata)"""
        result = {
            "scripted_error_xy_mm": self._err_xy,
            "scripted_error_angle_deg": self._err_deg,
            "scripted_wall_threshold_n": self._wall_threshold,
        }
        result.update(self._error_sample_metadata)
        result.update(self._in_hole_disturbance_metadata)
        return result

    def get_episode_metrics(self) -> Dict[str, Any]:
        """Return scalar approach/contact diagnostics for trace metadata."""
        return dict(self._episode_metrics)

    def set_event_callback(
        self,
        callback: Optional[Callable[[str, Dict[str, Any]], None]],
    ) -> None:
        """Set an optional live event sink used by direct HDF5 collection."""
        self._event_callback = callback

    def get_control_annotation(self) -> Dict[str, Any]:
        """Return the current phase and whether its command is expert action."""
        return {
            "scripted_phase_code": int(self._in_hole_phase_code),
            "expert_action_mask": int(self._expert_action_mask),
        }

    def _set_in_hole_phase(
        self,
        phase: str,
        extra: Optional[Dict[str, Any]] = None,
    ) -> None:
        phase_codes = {
            "inactive": 0,
            "insert": 1,
            "disturbance": 2,
            "recovery": 3,
            "complete": 4,
            "failed": 5,
        }
        phase = str(phase)
        if phase not in phase_codes:
            raise ValueError(f"unknown in-hole phase: {phase}")
        changed = phase != getattr(self, "_in_hole_phase", "inactive")
        self._in_hole_phase = phase
        self._in_hole_phase_code = phase_codes[phase]
        self._expert_action_mask = 0 if phase == "disturbance" else 1
        callback = getattr(self, "_event_callback", None)
        if changed and callable(callback):
            callback(
                f"scripted_in_hole_{phase}",
                dict(extra or {}),
            )

    def _sample_in_hole_disturbance(self) -> None:
        """Consume one deterministic in-hole disturbance coverage cell."""
        scheduler = self._in_hole_disturbance_scheduler
        if scheduler is None:
            self._in_hole_disturbance_sample = None
            self._in_hole_disturbance_metadata = {
                "scripted_in_hole_correction_enabled": 0,
            }
            return

        sample = scheduler.take()
        self._in_hole_disturbance_sample = sample
        metadata = sample.to_dict()
        metadata.update(
            {
                "scripted_in_hole_correction_enabled": 1,
                "scripted_in_hole_disturbance_coverage_size": scheduler.size,
                "scripted_in_hole_disturbance_coverage_order": scheduler.order,
                "scripted_in_hole_disturbance_coverage_seed": scheduler.seed,
                "scripted_disturbance_actions_are_expert": 0,
                "scripted_force_control_source": (
                    "gravity_compensated_world_wrench"
                ),
            }
        )
        self._in_hole_disturbance_metadata = metadata
        logger.info(
            "IN_HOLE sample: depth=%.0f%% direction=%.0fdeg amplitude=%.2fmm cell=%s",
            sample.depth_fraction * 100.0,
            sample.direction_deg,
            sample.amplitude_mm,
            metadata["scripted_in_hole_cell_label"],
        )

    def _sample_wall_threshold(self):
        """每条 episode 重新采样墙接触力阈值。"""
        f_min = float(self.cfg.get("wall_contact_force_min", 15.0))
        f_max = float(self.cfg.get("wall_contact_force_max", 28.0))
        if f_max < f_min:
            f_min, f_max = f_max, f_min
        self._wall_threshold = float(self.rng.uniform(f_min, f_max))
        logger.info(
            f"WALL_THRESHOLD: sampled {self._wall_threshold:.1f}N "
            f"from [{f_min:.1f}, {f_max:.1f}]N"
        )

    def _sample_error(self):
        """
        Sample the XZ rim-contact offset for one episode.

        The stratified mode consumes one deterministic radius/angle cell per
        attempt.  The legacy mode keeps the old fixed-radius/random-angle
        behavior for profiles that have not opted into coverage scheduling.
        """
        if self._error_scheduler is not None:
            sample = self._error_scheduler.take()
            sample_metadata = sample.to_dict()
            sample_metadata.update(
                {
                    "scripted_error_coverage_size": self._error_scheduler.size,
                    "scripted_error_coverage_order": self._error_scheduler.order,
                    "scripted_error_coverage_seed": self._error_scheduler.seed,
                }
            )
            m = float(sample.radius_mm)
            deg = float(sample.angle_deg)
            self._error_sample_metadata = sample_metadata
        else:
            m = float(self.cfg.get("scripted_error_radius_mm", 10.0))
            deg = float(self.rng.uniform(0, 360))
            self._error_sample_metadata = {
                "scripted_error_coverage_mode": "random_fixed_radius",
                "scripted_error_radius_mm": m,
            }
        x_mm = m * math.cos(math.radians(deg))
        z_mm = m * math.sin(math.radians(deg))
        self._err_xy = m
        self._err_deg = math.degrees(math.atan2(z_mm, x_mm))
        self._err_w = np.array([x_mm / 1000.0, 0, z_mm / 1000.0])
        q = "Q1右上" if (x_mm >= 0 and z_mm >= 0) else \
            "Q2左上" if (x_mm <  0 and z_mm >= 0) else \
            "Q3左下" if (x_mm <  0 and z_mm <  0) else "Q4右下"
        logger.info(f"ERROR: dx={x_mm:+.1f}mm  dz={z_mm:+.1f}mm  "
                    f"mag={m:.1f}mm  angle={self._err_deg:.0f}°  {q}")

    def _init_dofs(self):
        """Cache the moving task site and seven arm DOF addresses."""
        if self._site_id is not None:
            return
        self._site_id = mujoco.mj_name2id(
            self.rc.model, mujoco.mjtObj.mjOBJ_SITE, self.moving_site_name)
        if self._site_id == -1:
            raise ValueError(f"Moving task site not found: {self.moving_site_name}")
        self._arm_dofs = []
        for jn in self.rc.arm_joint_names:
            jid = mujoco.mj_name2id(self.rc.model, mujoco.mjtObj.mjOBJ_JOINT, jn)
            self._arm_dofs.append(int(self.rc.model.jnt_dofadr[jid]))

    # ==================================================================
    # Calibration — 6-direction central-difference + SVD → SO(3)
    # ==================================================================

    def _load_calib(self) -> bool:
        """
        从磁盘加载缓存的标定数据。

        如果文件存在且格式正确，加载 R_w_ik 和 t_w_ik 到类变量，
        后续所有 episode 不再需要重新标定。

        返回: True = 加载成功, False = 需要运行 _calibrate()
        """
        import os
        f = ScriptedPegInsertionController._calib_file
        if f is None or not os.path.exists(f):
            return False
        try:
            data = np.load(f)
            ScriptedPegInsertionController._R_w_ik = data["R"]
            ScriptedPegInsertionController._t_w_ik = data["t"]
            logger.info(f"CALIB: loaded from {f}")
            return True
        except Exception as e:
            logger.warning(f"CALIB: load failed ({e}), will re-calibrate")
            return False

    def _calibrate(self) -> bool:
        """
        标定 IK ↔ World 坐标系变换。

        方法：
          1. 移动机械臂到 IK 参考位姿，记录世界坐标 p_center
          2. 在 IK 空间沿 ±X, ±Y, ±Z 各走 20mm，记录世界位移
          3. 中心差分消除偏置: r = (p_+ - p_-) / (2×20mm)
          4. SVD 将 R_raw 投影到 SO(3) (最近的合法旋转矩阵)
          5. 平移: t = p_center - R @ p_ik_ref

        运行条件：
          保持恒定末端姿态，标定中所有探测点使用相同四元数。
          每个探测前先回中心再探，保证 IK 在同一连续分支。

        耗时：约 30 秒 (7个位置 × 1.5-2s 等待)
        """
        logger.info("CALIB: 6-dir probes (~30 s) …")

        # ================================================================
        # 1. 移动到 IK 参考位姿，记录世界 peg 位置作为"中心"
        # ================================================================
        j0 = self._ik_call(self._ik_ref, seed_joints=None)  # ROS IK 解参考位姿
        if j0 is None:
            logger.error("CALIB: cannot solve IK reference pose")
            return False
        self._set_wait(j0, 1.5)          # 移动并等待稳定
        p_center = self._peg_w()          # 记录世界位置
        logger.info(f"CALIB: centre peg_w = {[round(float(v), 5) for v in p_center]}")

        # ================================================================
        # 保存参考关节角作为固定 IK 种子 (标定探测时使用，
        # 避免 IK 跳解到其他分支)
        # ================================================================
        cj = self.rc.get_current_joints()  # 当前关节角 (外部约定)
        jn = self.rc.arm_joint_names
        as_ = self._arm_sign
        seed_joints = []
        for i, n in enumerate(jn):
            try:
                idx = list(self.rc.joint_names).index(n)
                seed_joints.append(float(cj[idx]) * float(as_[i]))
            except (ValueError, IndexError):
                seed_joints.append(0.0)
        ScriptedPegInsertionController._ik_ref_joints = seed_joints

        # ================================================================
        # 2. 6 方向探测，δ = 20 mm
        #    每个方向：先回中心 → 探测 → 记录世界位置
        # ================================================================
        delta = 0.020  # 20mm
        directions = [
            ("+X", [delta, 0, 0]), ("-X", [-delta, 0, 0]),
            ("+Y", [0, delta, 0]), ("-Y", [0, -delta, 0]),
            ("+Z", [0, 0, delta]), ("-Z", [0, 0, -delta]),
        ]
        probes: Dict[str, np.ndarray] = {}

        for label, d_ik in directions:
            self._set_wait(j0, 0.8)       # 先回中心 (保持同一 IK 分支)
            ik_tgt = self._ik_ref + np.array(d_ik)
            j = self._ik_call(ik_tgt, seed_joints=seed_joints)  # 固定种子
            if j is None:
                logger.error(f"CALIB: IK failed for {label}")
                return False
            self._set_wait(j, 1.0)        # 移动并等待稳定
            pw = self._peg_w()
            probes[label] = pw            # 记录世界位置
            logger.info(f"  CALIB {label}: ik={[round(v,4) for v in ik_tgt]}  "
                        f"→ world={[round(float(v),5) for v in pw]}")

        self._set_wait(j0, 1.0)  # 回到中心

        # ================================================================
        # 3. 中心差分 → R_raw
        #    r_x = (p_+X - p_-X) / (2δ)  → IK 的 X 轴在世界中的方向
        #    同理 r_y, r_z
        # ================================================================
        def _d(label_pos, label_neg):
            return (np.asarray(probes[label_pos]) -
                    np.asarray(probes[label_neg])) / (2.0 * delta)

        r_x = _d("+X", "-X")
        r_y = _d("+Y", "-Y")
        r_z = _d("+Z", "-Z")
        R_raw = np.column_stack([r_x, r_y, r_z])
        logger.info(f"CALIB: R_raw = \n{R_raw}")

        # ================================================================
        # 4. SVD → SO(3): 将近似旋转矩阵投影到最近合法旋转矩阵
        #    R_raw = U Σ V^T  →  R = U V^T
        #    如果 det(R) < 0 (反射)，翻转最后一个奇异向量
        # ================================================================
        U, s, Vt = np.linalg.svd(R_raw)
        R_w_ik = U @ Vt
        if np.linalg.det(R_w_ik) < 0:
            U[:, -1] *= -1
            R_w_ik = U @ Vt
        logger.info(f"CALIB: R_w_ik (SO(3)) = \n{R_w_ik}")

        # ================================================================
        # 5. 平移向量: t = p_center - R @ p_ik_ref
        #    物理含义：IK 坐标系原点在世界坐标系中的位置
        # ================================================================
        t_w_ik = np.asarray(p_center, dtype=np.float64) - R_w_ik @ self._ik_ref
        logger.info(f"CALIB: t_w_ik = {[round(float(v),5) for v in t_w_ik]}")

        # ================================================================
        # 6. 快速验证：用 R,t 预测中心位置 -> 残差应接近 0
        # ================================================================
        p_pred = R_w_ik @ self._ik_ref + t_w_ik
        err = float(np.linalg.norm(p_pred - np.asarray(p_center)))
        logger.info(f"CALIB: centre residual = {err*1000:.3f} mm")

        # ================================================================
        # 7. 缓存到类变量并保存到磁盘供后续进程使用
        # ================================================================
        ScriptedPegInsertionController._R_w_ik = R_w_ik
        ScriptedPegInsertionController._t_w_ik = t_w_ik
        try:
            import os
            f = ScriptedPegInsertionController._calib_file
            if f:
                os.makedirs(os.path.dirname(f), exist_ok=True)
                np.savez(f, R=R_w_ik, t=t_w_ik)
                logger.info(f"CALIB: saved to {f}")
        except Exception as e:
            logger.warning(f"CALIB: could not save ({e})")
        logger.info("CALIB done, cached.")
        return True

    # ==================================================================
    # APPROACH — 接近墙面
    # ==================================================================

    def _approach(self) -> Optional[EpisodeResult]:
        """
        Move to the offset rim target using measured site feedback.

        Free-space motion and near-contact probing use separate configured
        speeds.  A low debounced force detects contact; after detection a
        bounded additional push may build the requested contact force.  This
        avoids spending tens of seconds crawling through the final 20 mm and
        prevents an unbounded force-seeking command.
        """
        hole_e = self._hole_entrance()  # 孔口世界位置 (墙面处)
        contact_w = hole_e + self._err_w  # 孔口平面上的偏差接触目标
        standoff_m = float(self.cfg.get("approach_standoff_m", 0.030))
        pre_w = contact_w - self._insert_axis * standoff_m

        control_period = max(
            0.005, float(self.cfg.get("approach_control_period_s", 0.03))
        )
        force_poll_period = max(
            0.001, float(self.cfg.get("approach_force_poll_period_s", 0.005))
        )
        free_speed = float(self.cfg.get("approach_free_speed_m_s", 0.050))
        far_speed = float(self.cfg.get("approach_far_speed_m_s", 0.025))
        near_speed = float(self.cfg.get("approach_near_speed_m_s", 0.008))
        probe_speed = float(self.cfg.get("approach_probe_speed_m_s", 0.002))
        far_distance = float(self.cfg.get("approach_far_distance_m", 0.010))
        near_distance = float(self.cfg.get("approach_near_distance_m", 0.003))
        arrival_tolerance = float(
            self.cfg.get("approach_arrival_tolerance_m", 0.0005)
        )
        probe_activation_distance = float(
            self.cfg.get("approach_probe_activation_distance_m", 0.002)
        )
        detect_force = float(
            self.cfg.get("wall_contact_detect_force_n", 3.0)
        )
        detect_dwell = max(
            force_poll_period,
            float(self.cfg.get("wall_contact_detect_dwell_s", 0.015)),
        )
        target_dwell = max(
            force_poll_period,
            float(self.cfg.get("wall_contact_target_dwell_s", 0.005)),
        )
        max_push_m = max(
            0.0, float(self.cfg.get("wall_contact_max_push_m", 0.002))
        )
        contact_settle_s = max(
            control_period,
            float(self.cfg.get("wall_contact_settle_s", 0.30)),
        )

        if not (
            free_speed > 0.0
            and far_speed > 0.0
            and near_speed > 0.0
            and probe_speed > 0.0
        ):
            raise ValueError("all approach speeds must be positive")
        if not 0.0 < near_distance < far_distance:
            raise ValueError(
                "approach distances must satisfy 0 < near < far"
            )
        if not 0.0 < detect_force <= self._wall_threshold < self.FORCE_LIMIT:
            raise ValueError(
                "contact thresholds must satisfy 0 < detect <= target < overload"
            )

        d_total = float(
            np.linalg.norm(pre_w - self._peg_w())
            + np.linalg.norm(contact_w - pre_w)
        )
        target_offset_mm = float(np.linalg.norm(self._err_w[[0, 2]]) * 1000.0)
        logger.info(f"APPROACH: two-segment {d_total*1000:.0f}mm, "
                    f"pre={[round(float(v),4) for v in pre_w]} → "
                    f"contact={[round(float(v),4) for v in contact_w]}  "
                    f"offset_target={target_offset_mm:.2f}mm  "
                    f"detect={detect_force:.1f}N target={self._wall_threshold:.1f}N")

        approach_start = time.monotonic()
        peak_force = 0.0
        detect_accum = 0.0
        target_accum = 0.0
        contact_detected = False
        contact_push_m = 0.0
        probe_active = False
        probe_hold_elapsed = 0.0
        probe_command_w = None
        probe_command_travel = 0.0
        probe_max_travel = 0.0

        def finish_contact(target_reached: bool):
            pw = self._peg_w()
            offset_xz_mm = math.sqrt(
                (float(pw[0]) - float(hole_e[0])) ** 2
                + (float(pw[2]) - float(hole_e[2])) ** 2
            ) * 1000.0
            duration = time.monotonic() - approach_start
            self._episode_metrics.update(
                {
                    "scripted_approach_duration_s": float(duration),
                    "scripted_contact_detected": 1,
                    "scripted_contact_target_reached": int(target_reached),
                    "scripted_contact_actual_offset_mm": float(offset_xz_mm),
                    "scripted_contact_target_offset_mm": float(target_offset_mm),
                    "scripted_contact_offset_error_mm": float(
                        offset_xz_mm - target_offset_mm
                    ),
                    "scripted_contact_peak_force_n": float(peak_force),
                    "scripted_contact_push_depth_mm": float(contact_push_m * 1000.0),
                }
            )
            tol_mm = float(self.cfg.get("contact_offset_tolerance_mm", 1.0))
            if abs(offset_xz_mm - target_offset_mm) > tol_mm:
                logger.warning(
                    "APPROACH contact offset outside tolerance: "
                    f"actual={offset_xz_mm:.2f}mm target={target_offset_mm:.2f}mm "
                    f"tol={tol_mm:.2f}mm"
                )
            logger.info(
                "APPROACH contact complete: "
                f"duration={duration:.2f}s peak={peak_force:.1f}N "
                f"target_reached={target_reached} push={contact_push_m*1000:.2f}mm "
                f"offset={offset_xz_mm:.2f}/{target_offset_mm:.2f}mm"
            )
            return "contact"

        def move_segment(target_w: np.ndarray, detect_wall: bool, label: str):
            nonlocal peak_force, detect_accum, target_accum
            nonlocal contact_detected, contact_push_m
            nonlocal probe_active, probe_hold_elapsed
            nonlocal probe_command_w, probe_command_travel, probe_max_travel
            while True:
                if self._episode_timed_out():
                    return EpisodeResult(False, "timeout", Phase.APPROACH, 0, 0.0)
                if self._overload():
                    return EpisodeResult(False, "overload", Phase.APPROACH, 0, 0.0)

                actual_w = self._peg_w()
                delta = target_w - actual_w
                dr = float(np.linalg.norm(delta))
                if not detect_wall and dr <= arrival_tolerance:
                    return None

                if not detect_wall:
                    speed = free_speed
                    desired_w = actual_w + delta / max(dr, 1e-9) * min(
                        free_speed * control_period, dr
                    )
                else:
                    axial_gap = max(
                        0.0, float(np.dot(contact_w - actual_w, self._insert_axis))
                    )
                    if contact_detected or axial_gap <= probe_activation_distance:
                        if not probe_active:
                            probe_active = True
                            probe_command_w = actual_w.copy()
                            probe_max_travel = axial_gap + max_push_m
                        speed = probe_speed
                        advance_step = min(
                            probe_speed * control_period,
                            max(0.0, probe_max_travel - probe_command_travel),
                        )
                        if advance_step > 1e-12:
                            probe_command_travel += advance_step
                            probe_command_w = (
                                probe_command_w
                                + self._insert_axis * advance_step
                            )
                            contact_push_m = max(
                                0.0,
                                float(
                                    np.dot(
                                        probe_command_w - contact_w,
                                        self._insert_axis,
                                    )
                                ),
                            )
                            probe_hold_elapsed = 0.0
                        else:
                            probe_hold_elapsed += control_period
                        desired_w = probe_command_w.copy()
                        if probe_hold_elapsed >= contact_settle_s:
                            if contact_detected:
                                return finish_contact(target_reached=False)
                            self._episode_metrics.update(
                                {
                                    "scripted_approach_duration_s": float(
                                        time.monotonic() - approach_start
                                    ),
                                    "scripted_contact_detected": 0,
                                    "scripted_contact_peak_force_n": float(peak_force),
                                    "scripted_contact_push_depth_mm": float(
                                        contact_push_m * 1000.0
                                    ),
                                }
                            )
                            return EpisodeResult(
                                False,
                                "contact_not_detected",
                                Phase.APPROACH,
                                0,
                                0.0,
                            )
                    else:
                        if axial_gap > far_distance:
                            speed = far_speed
                        elif axial_gap > near_distance:
                            speed = near_speed
                        else:
                            speed = probe_speed
                        desired_w = actual_w + delta / max(dr, 1e-9) * min(
                            speed * control_period, dr
                        )

                q_int = self._mujoco_ik_step(desired_w, constrain_ori=True)
                if q_int is None:
                    return EpisodeResult(False, "ik", Phase.APPROACH, 0, 0.0)
                j = [float(q_int[i]) * float(self._arm_sign[i]) for i in range(7)]
                self.rc.set_arm_positions(j)

                poll_count = max(int(math.ceil(control_period / force_poll_period)), 1)
                for _ in range(poll_count):
                    time.sleep(force_poll_period)
                    ft = self._ft_world()
                    if ft is None:
                        continue

                    fn = float(np.linalg.norm(ft[:3]))
                    peak_force = max(peak_force, fn)
                    if fn > self.FORCE_LIMIT:
                        logger.error(f"OVERLOAD {fn:.0f}N")
                        return EpisodeResult(False, "overload", Phase.APPROACH, 0, 0.0)

                    if not detect_wall:
                        continue
                    if fn >= detect_force:
                        detect_accum += force_poll_period
                    else:
                        detect_accum = 0.0
                    if not contact_detected and detect_accum >= detect_dwell:
                        contact_detected = True
                        self._episode_metrics[
                            "scripted_contact_detect_time_s"
                        ] = float(time.monotonic() - approach_start)
                        logger.info(
                            f"APPROACH contact detected: |F|={fn:.1f}N "
                            f"after {self._episode_metrics['scripted_contact_detect_time_s']:.2f}s"
                        )

                    if contact_detected and fn >= self._wall_threshold:
                        target_accum += force_poll_period
                    else:
                        target_accum = 0.0
                    if contact_detected and target_accum >= target_dwell:
                        return finish_contact(target_reached=True)

                progress_key = (label, int(dr * 50))
                if progress_key != getattr(self, "_ld", None):
                    self._ld = progress_key
                    logger.info(
                        f"APPROACH {label}: d={dr*1000:.0f}mm "
                        f"speed={speed*1000:.1f}mm/s peak={peak_force:.1f}N "
                        f"probe={probe_active}"
                    )

        r = move_segment(pre_w, detect_wall=False, label="pre")
        if isinstance(r, EpisodeResult):
            return r

        r = move_segment(contact_w, detect_wall=True, label="contact")
        if isinstance(r, EpisodeResult):
            return r
        if r == "contact":
            return None
        return EpisodeResult(False, "contact_not_detected", Phase.APPROACH, 0, 0.0)

    # ==================================================================
    # Waypoint 对准 + 插入 (已知孔位置，直进直出)
    # ==================================================================

    def _build_waypoints(self, start_w: np.ndarray, hole_goal_w: np.ndarray,
                         num_align: Optional[int] = None,
                         num_insert: Optional[int] = None) -> list:
        """
        从回撤位置到孔底，生成一串世界坐标 waypoint。

        Phase 1 (XY 对准): 从回撤位置水平移到孔口正上方
        Phase 2 (Y 插入): 沿 -Y 方向插入到孔底

        返回: [wp0, wp1, ...]，每个 wp 是世界坐标 3D 位置
        """
        if num_align is None:
            num_align = int(self.cfg.get("waypoint_align_steps", 30))
        if num_insert is None:
            num_insert = int(self.cfg.get("waypoint_insert_steps", 45))

        waypoints = []

        # Phase 1: XY 对准（Z 也一起对齐）
        t1 = start_w.copy()
        t1[0] = float(hole_goal_w[0])
        t1[2] = float(hole_goal_w[2])

        for i in range(num_align):
            alpha = (i + 1) / num_align
            wp = start_w + alpha * (t1 - start_w)
            waypoints.append(wp)

        # Phase 2: 沿 -Y 插入到孔底
        for i in range(num_insert):
            alpha = (i + 1) / num_insert
            wp = t1 + alpha * (hole_goal_w - t1)
            waypoints.append(wp)

        logger.info(f"WAYPOINT: built {len(waypoints)} waypoints "
                    f"({num_align} align + {num_insert} insert)")
        return waypoints

    def _execute_waypoints(self, waypoints_w: list,
                           steps_per_wp: Optional[int] = None) -> Optional[EpisodeResult]:
        """
        对每个 waypoint 调 MuJoCo Jacobian IK 算关节角，发给 controller 执行。

        每步检查力过载和任务成功。

        返回: None = 成功完成, EpisodeResult = 失败
        """
        if steps_per_wp is None:
            steps_per_wp = int(self.cfg.get("waypoint_steps_per_wp", 90))

        align_steps = min(
            max(int(self.cfg.get("waypoint_align_steps", 30)), 0),
            len(waypoints_w),
        )
        insert_steps = max(len(waypoints_w) - align_steps, 1)
        disturbance_done = False

        for i, wp in enumerate(waypoints_w):
            if self._episode_timed_out():
                return EpisodeResult(False, "timeout", Phase.INSERT, 0, 0.0)
            if self._overload():
                return EpisodeResult(False, "overload", Phase.INSERT, 0, 0.0)

            # 如果 task_success 已经触发，提前结束
            if getattr(self.rc, "task_success_triggered", False):
                logger.info(f"WAYPOINT: task success triggered at wp {i}/{len(waypoints_w)}")
                self._set_in_hole_phase("complete")
                time.sleep(1.0)
                return None

            if i >= align_steps:
                self._set_in_hole_phase("insert")
                insert_progress = float(i - align_steps + 1) / float(insert_steps)
                sample = self._in_hole_disturbance_sample
                if (
                    not disturbance_done
                    and sample is not None
                    and insert_progress >= float(sample.depth_fraction)
                ):
                    disturbance_done = True
                    result = self._run_in_hole_disturbance(
                        nominal_w=np.asarray(wp, dtype=np.float64),
                        actual_depth_fraction=insert_progress,
                    )
                    if result is not None:
                        self._set_in_hole_phase(
                            "failed",
                            {"outcome": result.outcome},
                        )
                        return result
                    self._set_in_hole_phase("insert")

            q_int = self._mujoco_ik_step(wp, constrain_ori=True)
            if q_int is None:
                logger.warning(f"WAYPOINT: IK failed at wp {i}")
                return EpisodeResult(False, "ik", Phase.INSERT, 0, 0.0)

            ext = [float(q_int[j]) * float(self._arm_sign[j]) for j in range(7)]
            self.rc.set_arm_positions(ext)

            # 等待物理步进
            wait_s = steps_per_wp * self.rc.sim_timestep
            time.sleep(wait_s)

            # 每 10 个 waypoint 打一次日志
            if i % 10 == 0:
                p = self._peg_w()
                err = float(np.linalg.norm(p - wp))
                logger.info(f"WAYPOINT: {i}/{len(waypoints_w)} err={err*1000:.2f}mm")

        # 等最终稳定，检查 task_success
        time.sleep(0.5)
        if self._in_hole_disturbance_sample is not None and not disturbance_done:
            self._set_in_hole_phase(
                "failed",
                {"outcome": "in_hole_disturbance_not_reached"},
            )
            return EpisodeResult(
                False,
                "in_hole_disturbance_not_reached",
                Phase.INSERT,
                0,
                0.0,
            )
        self._set_in_hole_phase("complete")
        return None

    def _lateral_vector(self, vector_world: np.ndarray) -> np.ndarray:
        """Project a world-frame vector onto the insertion-normal plane."""
        vector = np.asarray(vector_world, dtype=np.float64).reshape(3)
        return vector - float(np.dot(vector, self._insert_axis)) * self._insert_axis

    def _disturbance_direction_world(self, angle_deg: float) -> np.ndarray:
        """Build a deterministic unit vector in the insertion-normal plane."""
        basis_x = self._lateral_vector(np.asarray([1.0, 0.0, 0.0]))
        if float(np.linalg.norm(basis_x)) <= 1e-9:
            basis_x = self._lateral_vector(np.asarray([0.0, 0.0, 1.0]))
        basis_x /= float(np.linalg.norm(basis_x))
        basis_z = np.cross(self._insert_axis, basis_x)
        basis_z /= float(np.linalg.norm(basis_z))
        angle = math.radians(float(angle_deg))
        return math.cos(angle) * basis_x + math.sin(angle) * basis_z

    def _run_in_hole_disturbance(
        self,
        nominal_w: np.ndarray,
        actual_depth_fraction: float,
    ) -> Optional[EpisodeResult]:
        """Inject one bounded lateral disturbance, then release it by force.

        The disturbance command is deliberately labelled non-expert.  Once a
        debounced lateral contact is detected, axial motion pauses and the
        gravity-compensated world-frame lateral force drives a bounded
        admittance correction.  Only the recovery command is expert action.
        """
        sample = self._in_hole_disturbance_sample
        if sample is None:
            return None

        control_period = max(
            0.005,
            float(self.cfg.get("in_hole_control_period_s", 0.03)),
        )
        ramp_steps = max(
            1,
            int(self.cfg.get("in_hole_disturbance_ramp_steps", 18)),
        )
        hold_steps = max(
            0,
            int(self.cfg.get("in_hole_disturbance_hold_steps", 4)),
        )
        detect_force = float(
            self.cfg.get("in_hole_contact_detect_force_n", 2.0)
        )
        detect_dwell = max(
            control_period,
            float(self.cfg.get("in_hole_contact_detect_dwell_s", 0.03)),
        )
        detection_min_fraction = float(
            self.cfg.get("in_hole_contact_detection_min_fraction", 0.70)
        )
        release_force = float(
            self.cfg.get("in_hole_contact_release_force_n", 1.5)
        )
        target_force = float(
            self.cfg.get("in_hole_contact_target_force_n", 5.0)
        )
        target_dwell = max(
            control_period,
            float(self.cfg.get("in_hole_contact_target_dwell_s", 0.03)),
        )
        release_dwell = max(
            control_period,
            float(self.cfg.get("in_hole_contact_release_dwell_s", 0.09)),
        )
        force_limit = float(self.cfg.get("in_hole_force_limit_n", 25.0))
        smoothing_alpha = float(
            self.cfg.get("in_hole_force_filter_alpha", 0.30)
        )
        correction_gain = float(
            self.cfg.get("in_hole_correction_gain_m_per_n", 0.00008)
        )
        correction_sign = float(
            self.cfg.get("in_hole_force_correction_sign", -1.0)
        )
        max_step = float(
            self.cfg.get("in_hole_correction_max_step_m", 0.00025)
        )
        max_travel = float(
            self.cfg.get("in_hole_correction_max_travel_m", 0.006)
        )
        correction_timeout = float(
            self.cfg.get("in_hole_correction_timeout_s", 3.0)
        )
        return_steps = max(
            1,
            int(self.cfg.get("in_hole_return_center_steps", 10)),
        )

        direction = self._disturbance_direction_world(sample.direction_deg)
        disturbance_m = float(sample.amplitude_mm) / 1000.0
        anchor = self._peg_w().copy()
        anchor += self._lateral_vector(nominal_w - anchor)
        disturbance_target = anchor + direction * disturbance_m
        started = time.monotonic()
        contact_detected = False
        detect_accum = 0.0
        contact_detect_time = None
        target_accum = 0.0
        target_reached = False
        peak_lateral = 0.0
        peak_total = 0.0
        filtered_lateral = np.zeros(3, dtype=np.float64)
        actual_amplitude_m = 0.0
        commanded_amplitude_m = 0.0

        self._set_in_hole_phase(
            "disturbance",
            {
                "depth_fraction": float(actual_depth_fraction),
                "direction_deg": float(sample.direction_deg),
                "amplitude_mm": float(sample.amplitude_mm),
                "expert_action": False,
            },
        )
        logger.info(
            "IN_HOLE disturbance: depth=%.1f%% direction=%.0fdeg amplitude=%.2fmm",
            actual_depth_fraction * 100.0,
            sample.direction_deg,
            sample.amplitude_mm,
        )

        for step_index in range(ramp_steps + hold_steps):
            if self._episode_timed_out():
                return EpisodeResult(False, "timeout", Phase.INSERT, 0, 0.0)
            if step_index < ramp_steps:
                alpha = float(step_index + 1) / float(ramp_steps)
                blend = 0.5 - 0.5 * math.cos(math.pi * alpha)
            else:
                blend = 1.0
            commanded_amplitude_m = blend * disturbance_m
            desired = anchor + blend * (disturbance_target - anchor)
            q_int = self._mujoco_ik_step(desired, constrain_ori=True)
            if q_int is None:
                return EpisodeResult(False, "ik", Phase.INSERT, 0, 0.0)
            command = [
                float(q_int[index]) * float(self._arm_sign[index])
                for index in range(7)
            ]
            self.rc.set_arm_positions(command)
            time.sleep(control_period)

            wrench = self._ft_world()
            if wrench is None:
                return EpisodeResult(
                    False,
                    "gravity_compensated_force_unavailable",
                    Phase.INSERT,
                    0,
                    0.0,
                )
            total_force = float(np.linalg.norm(wrench[:3]))
            lateral = self._lateral_vector(wrench[:3])
            lateral_force = float(np.linalg.norm(lateral))
            filtered_lateral = (
                smoothing_alpha * lateral
                + (1.0 - smoothing_alpha) * filtered_lateral
            )
            peak_total = max(peak_total, total_force)
            peak_lateral = max(peak_lateral, lateral_force)
            actual_amplitude_m = max(
                actual_amplitude_m,
                float(np.linalg.norm(self._lateral_vector(self._peg_w() - anchor))),
            )
            if total_force >= force_limit:
                self._freeze_current_arm("in-hole force limit")
                return EpisodeResult(
                    False,
                    "in_hole_force_limit",
                    Phase.INSERT,
                    0,
                    0.0,
                )
            # Gravity compensation removes static tool weight, not inertial
            # force from the deliberate ramp.  Do not classify early ramp
            # transients as wall contact; first command motion near the known
            # radial clearance, then apply the debounced force threshold.
            if (
                blend >= detection_min_fraction
                and lateral_force >= detect_force
            ):
                detect_accum += control_period
            else:
                detect_accum = 0.0
            if not contact_detected and detect_accum >= detect_dwell:
                contact_detected = True
                contact_detect_time = time.monotonic() - started
            if contact_detected and lateral_force >= target_force:
                target_accum += control_period
            else:
                target_accum = 0.0
            if contact_detected and target_accum >= target_dwell:
                target_reached = True
                break

        self._episode_metrics.update(
            {
                "scripted_in_hole_disturbance_actual_depth_fraction": float(
                    actual_depth_fraction
                ),
                "scripted_in_hole_disturbance_actual_amplitude_mm": float(
                    actual_amplitude_m * 1000.0
                ),
                "scripted_in_hole_disturbance_commanded_amplitude_mm": float(
                    commanded_amplitude_m * 1000.0
                ),
                "scripted_in_hole_contact_detected": int(contact_detected),
                "scripted_in_hole_contact_latency_s": float(
                    contact_detect_time
                    if contact_detect_time is not None
                    else time.monotonic() - started
                ),
                "scripted_in_hole_contact_target_reached": int(target_reached),
                "scripted_in_hole_contact_target_force_n": float(target_force),
                "scripted_in_hole_peak_lateral_force_n": float(peak_lateral),
                "scripted_in_hole_peak_total_force_n": float(peak_total),
            }
        )
        if not contact_detected:
            self._smooth_move_ee(
                target_w=anchor,
                total_steps=return_steps,
                step_wait_s=control_period,
                constrain_ori=True,
                phase=Phase.INSERT,
                log_prefix="IN_HOLE_NO_CONTACT_RETURN",
            )
            return EpisodeResult(
                False,
                "in_hole_disturbance_no_contact",
                Phase.INSERT,
                0,
                0.0,
            )

        self._set_in_hole_phase(
            "recovery",
            {
                "contact_lateral_force_n": float(peak_lateral),
                "expert_action": True,
            },
        )
        recovery_started = time.monotonic()
        release_accum = 0.0
        correction_travel = 0.0
        recovery_steps = 0
        recovery_success = False

        while time.monotonic() - recovery_started < correction_timeout:
            if self._episode_timed_out():
                return EpisodeResult(False, "timeout", Phase.INSERT, 0, 0.0)
            wrench = self._ft_world()
            if wrench is None:
                return EpisodeResult(
                    False,
                    "gravity_compensated_force_unavailable",
                    Phase.INSERT,
                    0,
                    0.0,
                )
            total_force = float(np.linalg.norm(wrench[:3]))
            lateral = self._lateral_vector(wrench[:3])
            lateral_force = float(np.linalg.norm(lateral))
            filtered_lateral = (
                smoothing_alpha * lateral
                + (1.0 - smoothing_alpha) * filtered_lateral
            )
            filtered_force = float(np.linalg.norm(filtered_lateral))
            peak_total = max(peak_total, total_force)
            peak_lateral = max(peak_lateral, lateral_force)
            if total_force >= force_limit:
                self._freeze_current_arm("in-hole recovery force limit")
                return EpisodeResult(
                    False,
                    "in_hole_force_limit",
                    Phase.INSERT,
                    0,
                    0.0,
                )

            if lateral_force <= release_force:
                release_accum += control_period
            else:
                release_accum = 0.0
            if release_accum >= release_dwell:
                recovery_success = True
                break

            excess_force = max(filtered_force - release_force, 0.0)
            if excess_force <= 0.0:
                time.sleep(control_period)
                continue
            step_m = min(correction_gain * excess_force, max_step)
            correction_travel += step_m
            if correction_travel > max_travel:
                break
            correction_direction = (
                correction_sign * filtered_lateral / max(filtered_force, 1e-9)
            )
            desired = self._peg_w() + correction_direction * step_m
            q_int = self._mujoco_ik_step(desired, constrain_ori=True)
            if q_int is None:
                return EpisodeResult(False, "ik", Phase.INSERT, 0, 0.0)
            command = [
                float(q_int[index]) * float(self._arm_sign[index])
                for index in range(7)
            ]
            self.rc.set_arm_positions(command)
            recovery_steps += 1
            time.sleep(control_period)

        recovery_duration = time.monotonic() - recovery_started
        self._episode_metrics.update(
            {
                "scripted_in_hole_recovery_success": int(recovery_success),
                "scripted_in_hole_recovery_duration_s": float(recovery_duration),
                "scripted_in_hole_recovery_steps": int(recovery_steps),
                "scripted_in_hole_correction_travel_mm": float(
                    correction_travel * 1000.0
                ),
                "scripted_in_hole_peak_lateral_force_n": float(peak_lateral),
                "scripted_in_hole_peak_total_force_n": float(peak_total),
            }
        )
        if not recovery_success:
            self._freeze_current_arm("in-hole recovery timeout")
            return EpisodeResult(
                False,
                "in_hole_recovery_failed",
                Phase.INSERT,
                0,
                0.0,
            )

        current = self._peg_w().copy()
        centered = current + self._lateral_vector(anchor - current)
        if not self._smooth_move_ee(
            target_w=centered,
            total_steps=return_steps,
            step_wait_s=control_period,
            constrain_ori=True,
            phase=Phase.INSERT,
            log_prefix="IN_HOLE_RETURN_CENTER",
        ):
            return EpisodeResult(False, "ik", Phase.INSERT, 0, 0.0)

        logger.info(
            "IN_HOLE recovered: duration=%.2fs travel=%.2fmm peak_lat=%.1fN",
            recovery_duration,
            correction_travel * 1000.0,
            peak_lateral,
        )
        return None

    # ==================================================================
    # ALIGN — ROS IK 粗对齐 + MuJoCo Jacobian 精对齐（legacy，不再由主流程调用）
    # ==================================================================

    def _align(self) -> Optional[EpisodeResult]:
        """
        阶段 2: 对齐 peg 到孔中心。

        三步走：
          Step 1: 回撤 3-8mm (沿世界 +Y)，离开墙面
          Step 2: ROS IK 粗对齐 (3 次)，可收敛到 ~3mm
          Step 3: MuJoCo Jacobian 精对齐 (6 次)，收敛到 < 1mm

        ROS IK 和 MuJoCo 模型间存在 ~3mm 固有偏差 (DH vs XML 运动学链)，
        所以必须用 MuJoCo 自己的 Jacobian 做最终精细对齐。
        """
        logger.info("ALIGN: ROS IK + MuJoCo Jacobian refinement")
        hole_g = self._hole_goal()  # 孔底 site 的世界位置
        R_w_ik = ScriptedPegInsertionController._R_w_ik
        t_w_ik = ScriptedPegInsertionController._t_w_ik

        # ==== Step 1: 回撤 3-8mm (世界 +Y) ====
        # 沿墙面法线方向退回，给后续对齐留操作空间
        retract_mm = float(self.rng.uniform(3.0, 8.0))  # 随机回撤量 (数据多样性)
        p = self._peg_w()
        rt = p.copy(); rt[1] += retract_mm / 1000.0
        ik_pos = R_w_ik.T @ (rt - t_w_ik)    # 世界 → IK
        j = self._ik_call(ik_pos, seed_joints=None)
        if j is None: return EpisodeResult(False, "ik", Phase.ALIGN, 0, 0.0)
        self.rc.set_arm_positions(j)
        time.sleep(0.5)
        logger.info(f"ALIGN: retracted {retract_mm:.1f}mm")

        # ==== Step 2: ROS IK 粗对齐 (最多 3 次) ====
        # 目标 = 孔中心 XZ + 当前 Y (保持回撤位置)
        for attempt in range(3):
            p = self._peg_w()
            err_xz = math.sqrt((hole_g[0] - p[0])**2 + (hole_g[2] - p[2])**2)
            logger.info(f"ALIGN-ik {attempt+1}: err_xz={err_xz*1000:.2f}mm")
            if err_xz < 0.001:
                logger.info("ALIGN converged")
                return None

            tgt = p.copy()
            tgt[0] = float(hole_g[0]); tgt[2] = float(hole_g[2])
            ik_pos = R_w_ik.T @ (tgt - t_w_ik)
            j = self._ik_call(ik_pos, seed_joints=None)
            if j is None: return EpisodeResult(False, "ik", Phase.ALIGN, 0, 0.0)
            self.rc.set_arm_positions(j)
            time.sleep(0.35)

        # ==== Step 3: MuJoCo Jacobian 精对齐 (最多 6 次) ====
        # 原因：ROS IK 的 DH 模型与 MuJoCo XML 模型有 ~3mm 固有偏差
        # MuJoCo Jacobian 使用仿真自己的模型，零偏差，可达到亚毫米精度
        logger.info("ALIGN: switching to MuJoCo Jacobian for fine alignment")
        for attempt in range(6):
            p = self._peg_w()
            err_xz = math.sqrt((hole_g[0] - p[0])**2 + (hole_g[2] - p[2])**2)
            logger.info(f"ALIGN-mj {attempt+1}: err_xz={err_xz*1000:.2f}mm")
            if err_xz < 0.001:
                logger.info("ALIGN converged")
                break

            tgt = p.copy()
            tgt[0] = float(hole_g[0]); tgt[2] = float(hole_g[2])
            # constrain_ori=True: 保持 peg 方向垂直于墙面 (6-DOF 雅可比)
            q_int = self._mujoco_ik_step(tgt, constrain_ori=True)
            if q_int is None:
                return EpisodeResult(False, "ik", Phase.ALIGN, 0, 0.0)
            # 内部约定 → 外部约定 (arm_sign 转换)
            ext = [float(q_int[i]) * float(self._arm_sign[i]) for i in range(7)]
            self.rc.set_arm_positions(ext)
            time.sleep(0.3)
        else:
            logger.warning(f"ALIGN residual: {err_xz*1000:.2f}mm")
        return None

    # ====================================================================
    # MuJoCo Jacobian IK (DLS) + 重力补偿
    # ====================================================================

    def _mujoco_ik_step(self, target_world: np.ndarray, max_iter: int = 80,
                         constrain_ori: bool = False):
        """
        基于 MuJoCo 局部 MjData 副本的 DLS 逆运动学 + 重力前馈补偿。

        为什么在局部副本上算？
          - 不污染真仿真状态
          - 雅可比和自我一致（同一模型的正向运动学和雅可比）
          - 零坐标系偏差

        constrain_ori=True 时使用 6×7 雅可比 (位置 + 完整姿态)，
        保持 peg tip site 的姿态等于本 episode 开始时记录的姿态。

        重力补偿：
          执行器稳态:  kp*(ctrl - qpos) = tau_g  (重力矩)
               → qpos = ctrl - tau_g/kp
          要使 qpos = q_ik (IK 解)，需要 ctrl = q_ik + tau_g/kp
          用 MuJoCo 的 mj_rne 在零加速度条件下求 tau_g，
          然后加到 IK 解上作为 ctrl 目标。

        返回: 内部约定关节角 (MuJoCo qpos)
        """
        nv = self.rc.model.nv           # 模型总 DOF 数
        n_arm = len(self._arm_dofs)     # 臂关节数 = 7
        target_xmat = self._ref_xmat if self._ref_xmat is not None else self._peg_xmat()

        # ==== 1. 复制当前仿真状态到局部 MjData ====
        with self.rc.lock:
            q0 = np.array([float(self.rc.data.qpos[d]) for d in self._arm_dofs])
            d = mujoco.MjData(self.rc.model)  # 局部副本
            d.qpos[:] = self.rc.data.qpos[:]  # 复制所有关节位置

        q = q0.copy()

        # ==== 2. DLS 迭代 ====
        for _ in range(max_iter):
            # 将当前关节角写入局部副本
            for j, dd in enumerate(self._arm_dofs):
                d.qpos[dd] = float(q[j])
            mujoco.mj_forward(self.rc.model, d)  # 正向运动学

            # ---- 位置误差 ----
            cur = d.site_xpos[self._site_id].copy()
            err_pos = target_world - cur

            # ---- 完整姿态误差: 保持 episode 初始 peg 姿态 ----
            if constrain_ori:
                xmat = d.site_xmat[self._site_id].copy().reshape(3, 3)
                # R_delta maps current site orientation to target orientation.
                # as_rotvec() gives the smallest rotation in the local/current
                # frame; multiplying by xmat expresses it in world coordinates,
                # matching MuJoCo's rotational site Jacobian convention.
                R_delta = xmat.T @ target_xmat
                err_ori = xmat @ R.from_matrix(R_delta).as_rotvec()
                err = np.concatenate([err_pos, err_ori])
            else:
                err = err_pos

            if float(np.linalg.norm(err)) < 1e-6:
                break  # 已收敛

            # ---- 构建雅可比矩阵 ----
            if constrain_ori:
                # 6×nv 完整雅可比 (位置 + 旋转)
                jac_p = np.zeros((3, nv))  # 平移
                jac_r = np.zeros((3, nv))  # 旋转
                mujoco.mj_jacSite(self.rc.model, d, jac_p, jac_r, self._site_id)
                J = np.zeros((6, n_arm))
                for j, dd in enumerate(self._arm_dofs):
                    J[0:3, j] = jac_p[:, int(dd)]
                    J[3:6, j] = jac_r[:, int(dd)]
            else:
                # 3×nv 位置雅可比
                jac = np.zeros((3, nv))
                mujoco.mj_jacSite(self.rc.model, d, jac, None, self._site_id)
                J = np.zeros((3, n_arm))
                for j, dd in enumerate(self._arm_dofs):
                    J[:, j] = jac[:, int(dd)]

            # ---- DLS: dq = J^T (J J^T + λI)^(-1) err ----
            JJT = J @ J.T
            JJT[np.diag_indices_from(JJT)] += 1e-4  # 阻尼 λ
            try:
                dq = J.T @ np.linalg.solve(JJT, err)
            except np.linalg.LinAlgError:
                dq = J.T @ np.linalg.lstsq(JJT, err, rcond=None)[0]

            # ---- 自适应步长 ----
            # 误差大时小步 (安全)，误差小时大步 (加速收敛)
            pos_err_norm = float(np.linalg.norm(err_pos))
            step = max(0.1, min(0.6, 1.0 - pos_err_norm / 0.005))
            q += step * dq

        # ==== 3. 重力补偿 ====
        # 将 IK 解写入局部副本
        for j, dd in enumerate(self._arm_dofs):
            d.qpos[dd] = float(q[j])
        d.qvel[:] = 0.0    # 零速度
        d.qacc[:] = 0.0    # 零加速度
        mujoco.mj_forward(self.rc.model, d)

        # mj_rne: 逆动力学。qacc=0, qvel=0 → 返回纯重力矩 tau_g
        tau_g = np.zeros(nv)
        mujoco.mj_rne(self.rc.model, d, 0, tau_g)

        # kp 值来自 XML 执行器定义 (motor_joint_1 ~ motor_joint_7)
        kp_vals = np.array([900., 900., 800., 700., 450., 250., 180.])

        # 补偿: ctrl = q_ik + tau_g/kp → 稳态 qpos = ctrl - tau_g/kp = q_ik
        q_comp = q.copy()
        for j, dd in enumerate(self._arm_dofs):
            q_comp[j] += float(tau_g[int(dd)]) / kp_vals[j]

        return [float(q_comp[i]) for i in range(n_arm)]

    # ==================================================================
    # INSERT — 重力补偿 Jacobian 推入 + 螺旋搜索
    # ==================================================================

    def _insert(self) -> Optional[EpisodeResult]:
        """
        阶段 3: 沿 -Y 方向推进 peg，遇到阻力时螺旋搜索。

        推进策略 (自适应步长):
          力 < 1N:   2mm/步 (松旷，快速推进)
          力 1-3N:   1mm/步 (轻微接触)
          力 3-5N:   0.5mm/步 (接近阻力上限)

        卡住检测:
          - 力 > 5N → 触发螺旋搜索
          - 10 步 Y 无进展 → 也触发螺旋 (力不高但 stuck)

        螺旋搜索:
          围绕孔中心 XZ 逐渐扩大搜索半径 (2.5mm → 5mm)，
          回撤 4mm 后移到搜索位置，再尝试推入。

        成功条件:
          系统 TaskSuccessAutoStop 触发 (peg tip 与 hole_goal 距离 < 3mm)
        """
        hg = self._hole_goal()
        stuck = 0              # 连续 stuck 计数
        phase = 0.0             # 螺旋相位角
        sr_mm = 2.5             # 螺旋搜索半径 (mm)
        step_count = 0          # 总步数
        last_y = float("inf")   # 上一拍 peg Y 位置
        no_progress = 0         # 无进展步数

        while True:
            # ---- 过载保护 ----
            if self._overload():
                return EpisodeResult(False, "overload", Phase.INSERT, 0, 0.0)

            # ---- 读取当前状态 ----
            p = self._peg_w()
            ft = self._ft_world()
            fn = float(np.linalg.norm(ft[:3])) if ft is not None else 0
            step_count += 1

            # ---- 成功: 系统 TaskSuccessAutoStop 触发 → 保持 2s ----
            if getattr(self.rc, "task_success_triggered", False):
                logger.info(f"INSERT done (system success, "
                            f"Y={p[1]:.4f}, |F|={fn:.1f}N, steps={step_count})")
                logger.info("INSERT: holding 2s before stop...")
                time.sleep(2.0)
                return None

            # ---- 追踪 Y 方向进展 ----
            dy = float(p[1] - last_y)
            if abs(dy) < 0.0002:
                no_progress += 1
            else:
                no_progress = 0
                last_y = float(p[1])

            # ---- 进度日志 (每 15 步) ----
            if step_count % 15 == 1:
                depth = (float(p[1]) - float(hg[1])) * 1000  # 剩余深度 (mm)
                _d3 = float(np.linalg.norm(p - hg))           # 3D 距离 (mm)
                logger.info(f"INSERT step#{step_count}: Y={p[1]:.4f} "
                            f"depth={depth:.1f}mm |F|={fn:.1f}N "
                            f"dist={_d3*1000:.1f}mm np={no_progress}")

            # ---- 螺旋触发: 高力 OR 卡住 ----
            _d3 = float(np.linalg.norm(p - hg))
            need_spiral = (fn > 5.0) or (no_progress > 10 and _d3 > 0.003)

            if need_spiral:
                # 回撤 4mm → 移到螺旋搜索位置
                target = p.copy()
                target[1] += 0.004  # 沿 +Y 回撤
                phase += 0.3         # 螺旋步进
                sr_mm = min(sr_mm + 0.3, 5.0)  # 扩大搜索半径 (上限 5mm)
                # 围绕孔中心螺旋
                target[0] = float(hg[0]) + (sr_mm / 1000) * math.cos(phase)
                target[2] = float(hg[2]) + (sr_mm / 1000) * math.sin(phase)
                stuck += 1
                no_progress = 0  # 重置卡住计数
                if stuck > 50:
                    return EpisodeResult(False, "stuck", Phase.INSERT, 0, 0.0)
            else:
                # 正常推进 (自适应步长)
                if fn < 1.0:
                    step_m = 0.002   # 2mm 大步
                elif fn < 3.0:
                    step_m = 0.001   # 1mm 中步
                else:
                    step_m = 0.0005  # 0.5mm 小步
                target = np.array([
                    float(hg[0]),
                    max(p[1] - step_m, float(hg[1]) - 0.005),  # 不超过孔底 5mm
                    float(hg[2]),
                ])
                stuck = 0
                sr_mm = max(sr_mm - 0.2, 1.0)  # 缩小搜索半径

            # ---- 重力补偿 Jacobian IK + 方向约束 ----
            q_int = self._mujoco_ik_step(target, constrain_ori=True)
            if q_int is None:
                return EpisodeResult(False, "ik", Phase.INSERT, 0, 0.0)
            ext = [float(q_int[i]) * float(self._arm_sign[i]) for i in range(7)]
            self.rc.set_arm_positions(ext)

            # ---- 力监控等待 ----
            # 在等待过程中也检查成功条件 (系统可能在物理稳定后触发)
            wait = 0.15 if fn < 1.0 else 0.25
            for _ in range(int(wait / 0.02)):
                time.sleep(0.02)
                ft_c = self._ft_world()
                if ft_c is not None and float(np.linalg.norm(ft_c[:3])) > self.FORCE_LIMIT:
                    logger.error(f"OVERLOAD {float(np.linalg.norm(ft_c[:3])):.0f}N")
                    return EpisodeResult(False, "overload", Phase.INSERT, 0, 0.0)
                # 等物理稳定后检查成功 (终端保持期间可能触发)
                if getattr(self.rc, "task_success_triggered", False):
                    logger.info(f"INSERT done mid-wait (system trigger, "
                                f"steps={step_count})")
                    logger.info("INSERT: holding 2s before stop...")
                    time.sleep(2.0)
                    return None

    # ==================================================================
    # 坐标转换: 世界 → IK
    # ==================================================================

    def _world_to_ik(self, p_w: np.ndarray) -> np.ndarray:
        """
        世界坐标 → IK 坐标。

        使用标定得到的 R_w_ik, t_w_ik:
          p_ik = R_w_ik^T @ (p_w - t_w_ik)
        """
        R_w_ik = ScriptedPegInsertionController._R_w_ik
        t_w_ik = ScriptedPegInsertionController._t_w_ik
        return R_w_ik.T @ (np.asarray(p_w, dtype=np.float64) - t_w_ik)

    def _ik_call(self, pos: np.ndarray, seed_joints=None):
        """
        调用 ROS IK 服务 /arm_teleop/right_arm_ik_srv 求逆解。

        参数:
          pos: IK 坐标系中的末端位置 [x, y, z]
          seed_joints: IK 求解种子关节角。None 则使用当前关节角。

        返回: 外部约定关节角 (7 个 float)，失败返回 None

        种子选择策略:
          - 标定时用固定种子 (避免跳解到其他肘构型)
          - APPROACH/ALIGN 用当前关节角 (相邻步的解相近)
        """
        from arm_teleop.srv import ArmIKRequest

        req = ArmIKRequest()
        req.method = self.ik_method  # "optimal_ref" 等

        # ---- 目标位姿 ----
        req.target_pose.position.x = float(pos[0])
        req.target_pose.position.y = float(pos[1])
        req.target_pose.position.z = float(pos[2])
        # 方向: 固定为参考方向 (来自 initial_robot_pose)
        req.target_pose.orientation.w = float(self._ik_q[0])
        req.target_pose.orientation.x = float(self._ik_q[1])
        req.target_pose.orientation.y = float(self._ik_q[2])
        req.target_pose.orientation.z = float(self._ik_q[3])

        req.current_arm_angle = 0.0
        # offset_list: 机械臂关节角度校准偏移量 (来自原始遥操作代码)
        req.offset_list = [0, -0.1, -0.2, -0.3, -0.4, -0.5,
                           0, 0.05, -0.05, 0.1, -0.1, 0.15, -0.15, 0.2, -0.2]
        req.offset_refer = 0.5

        # ---- IK 种子 ----
        if seed_joints is not None:
            req.init_joints = list(seed_joints)
        else:
            # 使用当前关节角 (外部约定) 作为种子
            as_ = self._arm_sign
            cj = self.rc.get_current_joints()
            jn = self.rc.arm_joint_names
            ij = []
            for i, n in enumerate(jn):
                try:
                    idx = list(self.rc.joint_names).index(n)
                    ij.append(float(cj[idx]) * float(as_[i]))
                except (ValueError, IndexError):
                    ij.append(0.0)
            req.init_joints = ij

        # ---- 调用 ROS 服务 ----
        try:
            resp = self.ik.call(req)
        except Exception as e:
            logger.error(f"IK exception: {e}")
            return None
        if not resp.success:
            logger.warning(f"IK success=False for pos={[round(v,4) for v in pos]}")
            return None
        return list(resp.solution)  # 外部约定关节角

    def _set_wait(self, joints, t):
        """发送关节角指令并等待 t 秒 (用于标定阶段)"""
        self.rc.set_arm_positions(joints)
        time.sleep(t)

    # ==================================================================
    # 退 peg: 插入成功后、机械臂复位前，先将 peg 拉出孔
    # ==================================================================

    def _retract_clear(self):
        """
        将 peg 沿世界 +Y 方向拉出 80mm，远离墙面。

        为什么直接写 command_joints？
          插入成功后 TaskSuccessAutoStop 进入 terminal_hold 状态，
          冻结了 command_joints。普通 set_arm_positions() 只写 target_joints，
          在 terminal_hold 期间被忽略 (physics_step 跳过 rate_limiter)。
          直接写 command_joints 绕过此冻结。

        为什么不用 reset_arm_to_initial_pose()？
          hard_set_qpos 瞬时跳变 qpos，没有重力补偿，
          机械臂会下垂并可能碰撞墙面。
        """
        logger.info("RETRACT: pulling peg out of hole...")
        p = self._peg_w()
        target = p.copy()
        target[1] += 0.080  # 沿 +Y 退 80mm

        for _ in range(4):
            q_int = self._mujoco_ik_step(target, constrain_ori=True)
            if q_int is None:
                logger.warning("RETRACT: IK failed, skipping")
                return
            # ---- 同时写 target_joints 和 command_joints ----
            # 绕过 terminal_hold 冻结，让执行器立即响应
            with self.rc.lock:
                self.rc.target_joints[:7] = q_int
                self.rc.command_joints[:7] = q_int
            time.sleep(0.35)
            p_now = self._peg_w()
            d = float(np.linalg.norm(p_now - target))
            if d < 0.005:  # 5mm 以内算到达
                logger.info("RETRACT: cleared")
                return
            if _ == 0:
                logger.info(f"RETRACT: target dist = {d*1000:.0f}mm")
        logger.info("RETRACT: done")

    # ==================================================================
    # 辅助函数
    # ==================================================================

    def _overload(self) -> bool:
        """检查力是否超过安全上限 (40N)"""
        ft = self._ft_world()
        if ft is not None and float(np.linalg.norm(ft[:3])) > self.FORCE_LIMIT:
            logger.error(f"OVERLOAD {float(np.linalg.norm(ft[:3])):.0f}N")
            return True
        return False

    def _freeze_current_arm(self, reason: str = "") -> None:
        """Freeze command/target at current qpos to avoid pushing into contact."""
        try:
            with self.rc.lock:
                q_now = np.array(
                    [float(self.rc.data.qpos[d]) for d in self._arm_dofs],
                    dtype=np.float64,
                )
                n = min(len(q_now), len(getattr(self.rc, "target_joints", [])))
                if n > 0:
                    self.rc.target_joints[:n] = q_now[:n].tolist()
                n = min(len(q_now), len(getattr(self.rc, "command_joints", [])))
                if n > 0:
                    self.rc.command_joints[:n] = q_now[:n].tolist()
                self.rc._apply_actuator_targets(self.rc.command_joints)
            suffix = f" ({reason})" if reason else ""
            logger.info(f"FREEZE current arm command{suffix}")
        except Exception as exc:
            logger.warning(f"FREEZE failed: {exc}")

    def _smooth_move_ee(self, target_w: np.ndarray, total_steps: int,
                        step_wait_s: float, constrain_ori: bool,
                        phase: Phase, log_prefix: str) -> bool:
        """Move peg tip to target_w through small Cartesian IK increments."""
        total_steps = max(int(total_steps), 1)
        start_w = self._peg_w().copy()
        target_w = np.asarray(target_w, dtype=np.float64)

        for i in range(total_steps):
            if self._episode_timed_out():
                return False
            if self._overload():
                return False

            alpha = float(i + 1) / float(total_steps)
            # cosine ease-in/ease-out，避免一开始和结束时目标速度突变
            s = 0.5 - 0.5 * math.cos(math.pi * alpha)
            wp = start_w + s * (target_w - start_w)

            q_int = self._mujoco_ik_step(wp, constrain_ori=constrain_ori)
            if q_int is None:
                logger.warning(f"{log_prefix}: IK failed at {i + 1}/{total_steps}")
                return False

            ext = [float(q_int[j]) * float(self._arm_sign[j]) for j in range(7)]
            self.rc.set_arm_positions(ext)
            time.sleep(max(float(step_wait_s), 0.0))

        err = float(np.linalg.norm(self._peg_w() - target_w))
        logger.info(f"{log_prefix}: done err={err*1000:.2f}mm")
        return True

    def _wait_force_release(self, threshold_n: float = 6.0,
                            timeout_s: float = 1.0) -> bool:
        """Wait briefly until contact force drops before starting alignment."""
        t0 = time.time()
        last_fn = 0.0
        while time.time() - t0 < timeout_s:
            ft = self._ft_world()
            last_fn = float(np.linalg.norm(ft[:3])) if ft is not None else 0.0
            if last_fn <= threshold_n:
                logger.info(f"FORCE_RELEASE: |F|={last_fn:.1f}N")
                return True
            time.sleep(0.02)

        logger.warning(
            f"FORCE_RELEASE: timeout, continue with |F|={last_fn:.1f}N "
            f"> {threshold_n:.1f}N"
        )
        return False

    def _peg_w(self) -> np.ndarray:
        """Return the configured moving task site's world position."""
        return np.asarray(
            self.rc.get_site_position(self.moving_site_name), dtype=np.float64
        )

    def _peg_xmat(self) -> np.ndarray:
        """读取 peg_tip_site 的完整世界姿态矩阵。"""
        with self.rc.lock:
            return self.rc.data.site_xmat[self._site_id].copy().reshape(3, 3)

    def _hole_goal(self) -> np.ndarray:
        """Return the configured fixed target goal site's world position."""
        return np.asarray(
            self.rc.get_site_position(self.target_goal_site_name),
            dtype=np.float64,
        )

    def _hole_entrance(self) -> np.ndarray:
        """Return the moving-site target at first task contact."""
        if self.target_approach_site_name:
            approach = self.rc.get_site_position(self.target_approach_site_name)
            if approach is None:
                raise ValueError(
                    "Target approach site not found: "
                    f"{self.target_approach_site_name}"
                )
            return np.asarray(approach, dtype=np.float64)

        # Legacy fixed-hole scene: derive the entrance plane from wall_task.
        w = self.rc.get_body_position(self.target_body_name)
        g = self._hole_goal()
        if w is None:
            return g - self.INSERT_AXIS_WORLD * 0.021
        e = g.copy(); e[1] = float(w[1])
        return e

    def _ft_world(self):
        """
        Return the gravity-compensated FT wrench in the world frame.

        Force-controlled phases intentionally have no raw-wrench fallback.
        This prevents tool weight or a missing compensation configuration
        from being interpreted as contact or a lateral correction signal.
        """
        getter = getattr(
            self.rc,
            "get_gravity_compensated_ft_wrench_world",
            None,
        )
        if not callable(getter):
            raise RuntimeError(
                "robot controller does not provide gravity-compensated "
                "world-frame FT wrench"
            )
        wrench = getter()
        if wrench is None:
            return None
        return np.asarray(wrench, dtype=np.float64).reshape(6)
