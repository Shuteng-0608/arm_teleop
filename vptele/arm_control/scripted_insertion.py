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
from typing import Any, Dict, List, Optional

import numpy as np
from scipy.spatial.transform import Rotation as R

from utils.logger import get_logger

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

        # 参考世界位置：标定后 peg 在 IK 参考位姿处的位置
        self._ref_w: Optional[np.ndarray] = None
        self._ref_xmat: Optional[np.ndarray] = None

        # 每个 episode 开始时会重新采样墙接触力阈值。
        self._wall_threshold = 0.0

        # ==================================================================
        # 标定缓存文件路径
        # 存储在 HDF5 数据目录下的 calib_ik_to_world.npz
        # 首次运行后缓存，后续启动直接加载，跳过 ~30s 的标定过程
        # ==================================================================
        import os
        config_dir = os.path.dirname(os.path.abspath(
            "/home/hmj/pangu/src/arm_teleop/vptele/config/config_arm_right_peg.yaml"))
        out_dir = os.path.join(config_dir,
                               self.cfg.get("hdf5_record_dir",
                                            "../../data/hole_random_60mm_hmj"))
        ScriptedPegInsertionController._calib_file = os.path.join(
            os.path.abspath(out_dir), "calib_ik_to_world.npz")

        # MuJoCo 对象缓存 (延迟初始化)
        self._site_id: Optional[int] = None    # peg_tip_site 的 MuJoCo ID
        self._arm_dofs: List[int] = []          # 7 个关节的 DOF 地址

    # ==================================================================
    # Public — 外部接口
    # ==================================================================

    def run_episode(self, error_xy_mm=None, error_angle_deg=None) -> EpisodeResult:
        """
        执行一条完整的 peg-in-hole 插入 episode。

        APPROACH(瞄偏撞墙, 力反馈) → 回撤 → waypoint 对准 → waypoint 插入。
        全程使用 MuJoCo Jacobian IK，不依赖 ROS IK 和标定。

        参数:
          error_xy_mm:      XY 偏移量 (mm)，None 则随机采样
          error_angle_deg:  XY 偏移方向 (°)，None 则随机采样

        返回:
          EpisodeResult: success=True 表示插入成功
        """
        t0 = time.time()
        self._sample_wall_threshold()

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

        # ---- 成功：退 peg 后再返回 (避免复位时撞墙) ----
        self._retract_clear()
        return EpisodeResult(True, "success", Phase.SUCCESS, 0, time.time() - t0)

    def get_last_error_info(self):
        """获取最近一次采样的 XY 误差信息 (用于写入 episode_metadata)"""
        return {
            "scripted_error_xy_mm": self._err_xy,
            "scripted_error_angle_deg": self._err_deg,
            "scripted_wall_threshold_n": self._wall_threshold,
        }

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
        随机生成 APPROACH 目标的 XZ 偏移。

        方法：在 XZ 平面独立均匀采样，保证四象限全覆盖。
        最小偏移 5mm (大于孔半径 ~4mm)，确保 peg 一定撞墙而非直接滑入孔。
        范围：[-7mm, +7mm] × [-7mm, +7mm]。
        """
        m = float(self.cfg.get("scripted_error_radius_mm", 10.0))
        deg = float(self.rng.uniform(0, 360))
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
        """惰性缓存 peg_tip_site ID 和 7 个关节的 DOF 地址 (模型不变，只查一次)"""
        if self._site_id is not None:
            return
        self._site_id = mujoco.mj_name2id(
            self.rc.model, mujoco.mjtObj.mjOBJ_SITE, "peg_tip_site")
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
        阶段 1: 从参考位姿逐步走向孔口 + 随机偏移位置。

        每步：用 MuJoCo Jacobian IK 算关节角 → set_arm_positions → 高频检查力
        力超过 _wall_threshold 后停止，进入后续回撤/waypoint 阶段。

        自适应步长：
          距离 > 80mm:  3mm/步  (快速接近)
          40-80mm:      1mm/步
          20-40mm:      0.3mm/步
          < 20mm:       0.1mm/步 (慢速精细)
        """
        hole_e = self._hole_entrance()  # 孔口世界位置 (墙面处)
        contact_w = hole_e + self._err_w  # 孔口平面上的偏差接触目标
        standoff_m = float(self.cfg.get("approach_standoff_m", 0.030))
        pre_w = contact_w.copy()
        pre_w[1] += standoff_m

        cur_w = self._ref_w.copy()       # 当前位置 = 参考位姿的世界位置
        d_total = float(np.linalg.norm(pre_w - cur_w) + np.linalg.norm(contact_w - pre_w))
        orig_start_w = self._ref_w.copy()
        orig_vec = contact_w - orig_start_w
        orig_len = float(np.linalg.norm(orig_vec))
        orig_dir = orig_vec / max(orig_len, 1e-9)
        target_offset_mm = float(np.linalg.norm(self._err_w[[0, 2]]) * 1000.0)
        logger.info(f"APPROACH: two-segment {d_total*1000:.0f}mm, "
                    f"pre={[round(float(v),4) for v in pre_w]} → "
                    f"contact={[round(float(v),4) for v in contact_w]}  "
                    f"offset_target={target_offset_mm:.2f}mm  "
                    f"wall_thresh={self._wall_threshold:.1f}N")

        def move_segment(target_w: np.ndarray, detect_wall: bool, label: str):
            nonlocal cur_w
            while True:
                # ---- 过载检查 ----
                if self._overload():
                    return EpisodeResult(False, "overload", Phase.APPROACH, 0, 0.0)

                # ---- 剩余距离 ----
                dr = float(np.linalg.norm(target_w - cur_w))
                if dr < 0.001:
                    return None

                # ---- 原 APPROACH 等效速度调度 ----
                # 两段式改变了当前 segment 的 target，直接用 dr 会让
                # contact 段一开始就落入慢速档。这里用当前点在原始
                # 单段 ref_w→contact_w 路径上的投影剩余距离来选择速度档，
                # 让速度表现更接近未改路径之前的 approach。
                progress = float(np.dot(cur_w - orig_start_w, orig_dir))
                progress = max(0.0, min(progress, orig_len))
                schedule_dr = max(orig_len - progress, 0.0)

                if schedule_dr > 0.080:      # > 80mm
                    step_m, wait_s = 0.003, 0.04
                elif schedule_dr > 0.040:    # 40-80mm
                    step_m, wait_s = 0.001, 0.08
                elif schedule_dr > 0.020:    # 20-40mm
                    step_m, wait_s = 0.0003, 0.12
                else:               # < 20mm
                    step_m, wait_s = 0.0001, 0.15

                # ---- 沿当前 segment 方向走一步 ----
                cur_w = cur_w + (target_w - cur_w) / dr * step_m
                q_int = self._mujoco_ik_step(cur_w, constrain_ori=True)
                if q_int is None:
                    return EpisodeResult(False, "ik", Phase.APPROACH, 0, 0.0)
                j = [float(q_int[i]) * float(self._arm_sign[i]) for i in range(7)]
                self.rc.set_arm_positions(j)

                # ---- 高频力检查 (每 10ms 一次)，沿用原节奏 ----
                for _ in range(max(int(wait_s / 0.01), 1)):
                    time.sleep(0.01)
                    ft = self._ft_world()
                    if ft is None:
                        continue

                    fn = float(np.linalg.norm(ft[:3]))
                    if fn > self.FORCE_LIMIT:
                        logger.error(f"OVERLOAD {fn:.0f}N")
                        return EpisodeResult(False, "overload", Phase.APPROACH, 0, 0.0)

                    if detect_wall and fn > self._wall_threshold:
                        # 触墙 → 停止 APPROACH，进入后续阶段
                        pw = self._peg_w()
                        offset_xz_mm = math.sqrt(
                            (float(pw[0]) - float(hole_e[0])) ** 2
                            + (float(pw[2]) - float(hole_e[2])) ** 2
                        ) * 1000.0
                        tol_mm = float(self.cfg.get("contact_offset_tolerance_mm", 1.0))
                        offset_msg = (
                            f"offset_xz={offset_xz_mm:.2f}mm "
                            f"target={target_offset_mm:.2f}mm"
                        )
                        if abs(offset_xz_mm - target_offset_mm) > tol_mm:
                            logger.warning(
                                f"APPROACH contact offset outside tolerance: {offset_msg}, "
                                f"tol={tol_mm:.2f}mm"
                            )
                        logger.info(
                            f"APPROACH: wall |F|={fn:.1f}N at "
                            f"{[round(float(v),4) for v in pw]} ({offset_msg})")
                        return "contact"

                # ---- 进度日志 ----
                progress_key = (label, int(dr * 100))
                if progress_key != getattr(self, "_ld", None):
                    self._ld = progress_key
                    logger.info(
                        f"APPROACH {label}: d={dr*1000:.0f}mm "
                        f"sched={schedule_dr*1000:.0f}mm"
                    )

        r = move_segment(pre_w, detect_wall=False, label="pre")
        if isinstance(r, EpisodeResult):
            return r

        r = move_segment(contact_w, detect_wall=True, label="contact")
        if isinstance(r, EpisodeResult):
            return r
        if r == "contact":
            return None

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

        for i, wp in enumerate(waypoints_w):
            if self._overload():
                return EpisodeResult(False, "overload", Phase.INSERT, 0, 0.0)

            # 如果 task_success 已经触发，提前结束
            if getattr(self.rc, "task_success_triggered", False):
                logger.info(f"WAYPOINT: task success triggered at wp {i}/{len(waypoints_w)}")
                time.sleep(1.0)
                return None

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
        """读取 peg_tip_site 的世界位置 (MuJoCo mj_forward 实时计算)"""
        return np.asarray(self.rc.get_site_position("peg_tip_site"), dtype=np.float64)

    def _peg_xmat(self) -> np.ndarray:
        """读取 peg_tip_site 的完整世界姿态矩阵。"""
        with self.rc.lock:
            return self.rc.data.site_xmat[self._site_id].copy().reshape(3, 3)

    def _hole_goal(self) -> np.ndarray:
        """读取 hole_goal_site 的世界位置 (孔底/目标点)"""
        return np.asarray(self.rc.get_site_position("hole_goal_site"), dtype=np.float64)

    def _hole_entrance(self) -> np.ndarray:
        """孔口世界位置 = hole_goal 的 XZ + wall_task body 的 Y (墙面位置)"""
        w = self.rc.get_body_position("wall_task")
        g = self._hole_goal()
        if w is None:
            return g - self.INSERT_AXIS_WORLD * 0.021
        e = g.copy(); e[1] = float(w[1])
        return e

    def _ft_world(self):
        """
        读取 FT 力传感器值，并转换到世界坐标系。

        MuJoCo 力传感器返回的是传感器局部坐标系的值，
        需要乘以 site_xmat (传感器 site 的旋转矩阵) 转到世界坐标系。
        这样 Fx, Fy, Fz 分别对应世界 X, Y, Z 方向的力。
        """
        w = self.rc.get_peg_ft_sensor()
        if w is None:
            return None
        w = np.asarray(w, dtype=np.float64)
        sid = mujoco.mj_name2id(self.rc.model,
                                mujoco.mjtObj.mjOBJ_SITE, "ft_sensor_site")
        if sid >= 0:
            with self.rc.lock:
                R_ = self.rc.data.site_xmat[sid].copy().reshape(3, 3)  # 传感器 → 世界
            w[:3] = R_ @ w[:3]    # 力旋转
            w[3:6] = R_ @ w[3:6]  # 力矩旋转
        return w
