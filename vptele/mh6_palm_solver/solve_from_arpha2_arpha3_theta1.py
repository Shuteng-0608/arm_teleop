"""
给定 arpha2, arpha3, theta1 (deg)，显式求解 theta2, theta3, arpha1 (deg)。

变量命名：
  arpha2 (已知) - Wb->W1 的 Rz 转角 (R1 = Ry(80)*Rz(arpha2))
  arpha3 (已知) - W4->W5 的 Rz 转角 (R5 = Ry(20)*Rz(arpha3))
  theta1 (已知) - W1->W2 的 Rz 转角 (R2 = Ry(-80)*Rz(theta1))
  theta2 (待求) - W2->W3 的 Rz 转角 (R3 = Ry(80)*Rz(theta2))
  theta3 (待求) - W3->W4 的 Rz 转角 (R4 = Ry(35)*Rz(theta3))
  arpha1 (待求) - W5->Wb 的 Rz 转角 (R6 = Ry(225)*Rz(arpha1))

核心推导:
  约束 R1*R2*R3*R4*R5*R6 = I
  -> R3*R4*R5*R6 = (R1*R2)^T = P

  取 C = Ry(-80) * P
  -> C = Rz(theta2)*Ry(35)*Rz(theta3)*Ry(20)*Rz(arpha3)*Ry(225)*Rz(arpha1)

  Rz(arpha1) 不影响第三列:
  C[:,2] = Rz(t2)*Ry(35)*Rz(t3)*Ry(20)*Rz(a3)*Ry(225)*[0,0,1]^T

  令 v = Ry(20)*Rz(a3)*Ry(225)*[0,0,1]^T  (已知)
  w = Ry(35)*Rz(t3)*v
  C[:,2] = Rz(t2)*w

  C[2,2] = w_z = -sin35*(v_x*cos(t3) - v_y*sin(t3)) + cos35*v_z
  -> cos(t3 + φ) = (cos35*v_z - C[2,2]) / (sin35 * sqrt(v_x^2+v_y^2))
  其中 φ = atan2(v_y, v_x)

  已知 t3 -> theta2 = atan2(C[1,2]*w_x - C[0,2]*w_y,
                            C[0,2]*w_x + C[1,2]*w_y)

  已知 t2,t3 -> arpha1 = atan2(J[1,0], J[1,1])
  其中 J = Ry(-225)*(R1*R2*R3*R4*R5)^T

归一化输入映射 (基于电机标定):
  u1,u2,u3 ∈ [0,1] -> arpha2 = 121.9*u1 - 31.1     [-31.1°, 90.8°]
                       arpha3 = -239*u2 + 59         [-180°, 59°]
                       theta1 = -32.3*u3 + 8.6       [-23.7°, 8.6°]

物理限位 (基于电机标定):
  arpha2  ∈ [-31.1, 90.8]   (输入硬限位)
  arpha3  ∈ [-180, 59]      (输入硬限位)
  theta1  ∈ [-23.7, 8.6]    (输入硬限位)
  theta2  ∈ 无限制
  arpha1  ∈ 无限制

电机输入值换算 (三组独立标定):
  arpha1: 500 + (99.0/23.6) * arpha1      0°→500, -23.6°→401
  arpha2: 500 - (380.0/90.8) * arpha2      0°→500, 90.8°→120   (原始 arpha2)
  arpha3: 247 - (753.0/180.0) * arpha3     0°→247, -180°→1000

输出约定:
  arpha2* = -arpha2 （即输出时 arpha2 取负）

用法:
  from mh6_palm_solver.solve_from_arpha2_arpha3_theta1 import MH6PalmSolver

  solver = MH6PalmSolver()

  # 电机值输出（推荐）
  solutions = solver.solve_motor(arpha2, arpha3, theta1)        # 角度输入
  solutions = solver.solve_motor_from_normalized(u1, u2, u3)    # [0,1]归一化输入
  # 返回 [[motor1, motor2, motor3], ...]

  # 角度输出
  solutions = solver.solve_arpha(arpha2, arpha3, theta1)        # 角度输入
  solutions = solver.solve_arpha_from_normalized(u1, u2, u3)    # [0,1]归一化输入
  # 返回 [[arpha1, arpha2*, arpha3], ...]
"""
import math
import numpy as np


class MH6PalmSolver:

    def __init__(self):
        # 常量
        self.C35 = math.cos(math.radians(35))
        self.S35 = math.sin(math.radians(35))
        self.C20 = math.cos(math.radians(20))
        self.S20 = math.sin(math.radians(20))
        self.C225 = math.cos(math.radians(225))
        self.S225 = math.sin(math.radians(225))

        self.P1 = self.p_from_RyTz(-61.83241, 56.9724)
        self.P2 = self.p_from_RyTz(-39.03855, 76.61523)
        self.P3 = self.p_from_RyTz(125.90102, 67.19535)
        self.P4 = self.p_from_RyTz(114.76245, 32.21445)
        self.P5 = self.p_from_RyTz(135.16303, 19.2719)
        self.P6 = self.p_from_RyTz(119.75471, 29.11805)

    def RyRz(self, phi_deg, psi_deg):
        """Ry(phi)*Rz(psi) rotation matrix"""
        cp = math.cos(math.radians(phi_deg))
        sp = math.sin(math.radians(phi_deg))
        cq = math.cos(math.radians(psi_deg))
        sq = math.sin(math.radians(psi_deg))
        return np.array([
            [cp*cq, -cp*sq,  sp],
            [   sq,     cq,   0],
            [-sp*cq,  sp*sq,  cp]
        ])

    def p_from_RyTz(self, phi_deg, d):
        sa = math.sin(math.radians(phi_deg))
        ca = math.cos(math.radians(phi_deg))
        return np.array([d * sa, 0, d * ca])

    def compute_translation_error(self, arpha2_deg, arpha3_deg, arpha1_deg, theta1_deg, theta2_deg, theta3_deg):
        """计算平移约束误差 |p_total|"""
        R1 = self.RyRz(80, arpha2_deg)
        R2 = self.RyRz(-80, theta1_deg)
        R3 = self.RyRz(80, theta2_deg)
        R4 = self.RyRz(35, theta3_deg)
        R5 = self.RyRz(20, arpha3_deg)
        R6 = self.RyRz(225, arpha1_deg)
        R12 = R1 @ R2; R123 = R12 @ R3; R1234 = R123 @ R4; R12345 = R1234 @ R5
        p_total = self.P1 + R1 @ self.P2 + R12 @ self.P3 + R123 @ self.P4 + R1234 @ self.P5 + R12345 @ self.P6
        return np.linalg.norm(p_total)

    def check_theta1_range(self, t1_deg, verbose=False):
        """Check theta1 input limit [-23.7, 8.6]"""
        ok = -23.7 <= t1_deg <= 8.6
        if verbose:
            print(f"  theta1 = {t1_deg:.2f} deg -> {'OK' if ok else 'OUT OF RANGE'} (limit [-23.7, 8.6])")
        return ok

    def _norm_to_180(self, deg):
        return (deg + 180) % 360 - 180

    def check_arpha2_range(self, a2_deg, verbose=False):
        """Check arpha2 input limit [-31.1, 90.8]"""
        v = self._norm_to_180(a2_deg)
        ok = -31.1 <= v <= 90.8
        if verbose:
            print(f"  arpha2 = {v:.2f} deg -> {'OK' if ok else 'OUT OF RANGE'} (limit [-31.1, 90.8])")
        return ok

    def check_arpha3_range(self, a3_deg, verbose=False):
        """Check arpha3 input limit [-180, 59]"""
        v = self._norm_to_180(a3_deg)
        ok = -180 <= v <= 59
        if verbose:
            print(f"  arpha3 = {v:.2f} deg -> {'OK' if ok else 'OUT OF RANGE'} (limit [-180, 59])")
        return ok

    def map_normalized(self, u1, u2, u3):
        """
        将 [0,1]^3 映射到物理角度值（基于三组电机标定数据）。

        参数:
          u1: arpha2 归一化输入 [0,1] -> [-31.1, 90.8]
          u2: arpha3 归一化输入 [0,1] -> [-180, 59]
          u3: theta1 归一化输入 [0,1] -> [-23.7, 8.6]

        返回:
          (arpha2, arpha3, theta1) in degrees
        """
        arpha2 = 121.9 * u1 - 31.1
        arpha3 = -239 * u2 + 59
        theta1 = -32.3 * u3 + 8.6
        return arpha2, arpha3, theta1

    def solve_remaining(self, arpha2_deg, arpha3_deg, theta1_deg):
        """
        给定 arpha2, arpha3, theta1, 求解 theta2, theta3, arpha1.

        返回 [(theta2, theta3, arpha1, rotation_error, translation_error), ...]
        按旋转误差排序。若无解返回 [].
        """
        # ---- theta1 physical limit check ----
        if not self.check_theta1_range(theta1_deg):
            return []

        # ---- arpha2/arpha3 input limit check ----
        if not self.check_arpha2_range(arpha2_deg) or not self.check_arpha3_range(arpha3_deg):
            return []

        # ---- known rotations ----
        R1 = self.RyRz(80, arpha2_deg)
        R2 = self.RyRz(-80, theta1_deg)
        R5 = self.RyRz(20, arpha3_deg)

        # ---- P = (R1*R2)^T ----
        R12 = R1 @ R2
        P_mat = R12.T

        # ---- C = Ry(-80) * P ----
        Ry_neg80 = self.RyRz(-80, 0)
        C = Ry_neg80 @ P_mat

        # ---- v = Ry(20)*Rz(arpha3)*Ry(225)*[0,0,1]^T ----
        e3 = np.array([0.0, 0.0, 1.0])
        s3 = self.RyRz(225, 0) @ e3  # Ry(225)*[0,0,1]^T
        u = self.RyRz(0, arpha3_deg) @ s3  # Rz(arpha3)*s3
        v = self.RyRz(20, 0) @ u  # Ry(20)*u

        vx, vy, vz = v[0], v[1], v[2]

        # ---- solve for theta3 ----
        Rv = math.hypot(vx, vy)
        if Rv < 1e-15:
            return []
        phi = math.atan2(vy, vx)

        val = (self.C35 * vz - C[2, 2]) / (self.S35 * Rv)
        if abs(val) > 1.0 + 1e-12:
            return []
        val = max(-1.0, min(1.0, val))

        phi_t3 = math.acos(val)
        t3_candidates = [-phi + phi_t3, -phi - phi_t3]

        results = []
        for t3_val in t3_candidates:
            t3_deg = math.degrees(t3_val) % 360

            # ---- w = Ry(35)*Rz(t3)*v ----
            Rz_t3 = self.RyRz(0, t3_deg)
            w = self.RyRz(35, 0) @ (Rz_t3 @ v)
            wx, wy = w[0], w[1]

            # ---- theta2 = atan2(...) ----
            cx, cy = C[0, 2], C[1, 2]
            denom = cx * wx + cy * wy
            numer = cy * wx - cx * wy
            t2_val = math.atan2(numer, denom)
            t2_deg = math.degrees(t2_val)
            t2_deg_norm = (t2_deg + 180) % 360 - 180

            # ---- arpha1 = atan2(J[1,0], J[1,1]) ----
            R3 = self.RyRz(80, t2_deg)
            R4 = self.RyRz(35, t3_deg)
            R12345 = R1 @ R2 @ R3 @ R4 @ R5
            J = self.RyRz(-225, 0) @ R12345.T
            a1_val = math.atan2(J[1, 0], J[1, 1])
            a1_deg = math.degrees(a1_val)
            a1_deg_norm = (a1_deg + 180) % 360 - 180

            # ---- verify rotation ----
            R6 = self.RyRz(225, a1_deg)
            Rloop = R1 @ R2 @ R3 @ R4 @ R5 @ R6
            rot_err = np.max(np.abs(Rloop - np.eye(3)))

            # ---- verify translation ----
            trans_err = self.compute_translation_error(
                arpha2_deg, arpha3_deg, a1_deg, theta1_deg, t2_deg, t3_deg)

            results.append((t2_deg_norm, t3_deg, a1_deg_norm, rot_err, trans_err))

        # dedup + sort by error
        unique = []
        for r in results:
            if not any(abs(r[0]-u[0])<1e-6 and abs(r[1]-u[1])<1e-6 and abs(r[2]-u[2])<1e-6 for u in unique):
                unique.append(r)
        unique.sort(key=lambda x: x[3])
        return unique

    def solve_from_normalized(self, u1, u2, u3):
        """
        从归一化输入 [0,1]³ 直接求解。

        返回 [(theta2, theta3, arpha1, rot_err, trans_err), ...]
        """
        a2, a3, t1 = self.map_normalized(u1, u2, u3)
        return self.solve_remaining(a2, a3, t1)

    def solve_arpha(self, arpha2_deg, arpha3_deg, theta1_deg):
        """
        角度输入，仅输出 arpha 三元组，不含误差信息。

        返回: [[arpha1, arpha2, arpha3], ...]  每个解一个三元组
        """
        raw = self.solve_remaining(arpha2_deg, arpha3_deg, theta1_deg)
        return [[self._norm_to_180(r[2]), -arpha2_deg, arpha3_deg] for r in raw]

    def solve_arpha_from_normalized(self, u1, u2, u3):
        """
        归一化输入，仅输出 arpha 三元组，不含误差信息。

        返回: [[arpha1, arpha2, arpha3], ...]  每个解一个三元组
        """
        a2, a3, t1 = self.map_normalized(u1, u2, u3)
        return self.solve_arpha(a2, a3, t1)

    # ---- 电机值转换 ----

    def solve_motor(self, arpha2_deg, arpha3_deg, theta1_deg):
        """
        角度输入，输出三个电机的输入值。

        注意: arpha2 用原始角度（未取负）计算电机值。

        校准:
          arpha1: 0°→500, -23.6°→401
          arpha2: 0°→500, 90.8°→120   (原始 arpha2，非 arpha2*)
          arpha3: 0°→247, -180°→1000

        返回: [[motor1, motor2, motor3], ...]  每个解一个三元组
        """
        raw = self.solve_arpha(arpha2_deg, arpha3_deg, theta1_deg)
        results = []
        for a1, a2_star, a3 in raw:
            # a1 = arpha1 (已归一化到 -180~180)
            # a2_star = -arpha2 (已取负)
            # a3 = arpha3
            m1 = 500 + (99.0 / 23.6) * a1
            m2 = 500 - (380.0 / 90.8) * arpha2_deg  # 用原始 arpha2
            m3 = 247 - (753.0 / 180.0) * arpha3_deg
            results.append([round(m1, 4), round(m2, 4), round(m3, 4)])
        return results

    def solve_motor_from_normalized(self, u1, u2, u3):
        """
        归一化输入，输出三个电机的输入值。

        返回: [[motor1, motor2, motor3], ...]  每个解一个三元组
        """
        a2, a3, t1 = self.map_normalized(u1, u2, u3)
        return self.solve_motor(a2, a3, t1)


# ============================================================
if __name__ == "__main__":
    print("=" * 60)
    print("arpha2,arpha3,theta1 -> theta2,theta3,arpha1 solver test")
    print("=" * 60)

    mh6_solver = MH6PalmSolver()

    # Test 1: near-zero
    print("\nTest 1: near-zero (arpha2=0, arpha3=0, theta1=0)")
    sols = mh6_solver.solve_remaining(0, 0, 0)
    for t2, t3, a1, rot_err, trans_err in sols:
        print(f"  theta2={t2:.4f} deg, theta3={t3:.4f} deg, arpha1={a1:.4f} deg  rot_err={rot_err:.2e}  |p|={trans_err:.2e}")
    if not sols:
        print("  (no solution)")

    # Test 2: normalized input (0,0,0)
    print("\nTest 2: normalized (u1=0, u2=0, u3=0)")
    sols = mh6_solver.solve_from_normalized(0.0, 0.0, 0.0)
    for t2, t3, a1, rot_err, trans_err in sols:
        print(f"  theta2={t2:.4f} deg, theta3={t3:.4f} deg, arpha1={a1:.4f} deg  rot_err={rot_err:.2e}  |p|={trans_err:.2e}")
    if not sols:
        print("  (no solution)")

    # Test 3: SolidWorks verification
    print("\nTest 3: SolidWorks (arpha2=47, arpha3=-80, theta1=-20)")
    sols = mh6_solver.solve_remaining(47, -80, -20)
    for t2, t3, a1, rot_err, trans_err in sols:
        t2n = t2 if t2 <= 180 else t2 - 360
        a1n = a1 if a1 <= 180 else a1 - 360
        print(f"  theta2={t2n:.4f} deg, theta3={t3:.4f} deg, arpha1={a1n:.4f} deg  rot_err={rot_err:.2e}  |p|={trans_err:.2e}")
    if not sols:
        print("  (no solution)")

    # Test 4: simplified output solve_arpha
    print("\nTest 4: solve_arpha (arpha2=47, arpha3=-80, theta1=-20)")
    arpha_sols = mh6_solver.solve_arpha(47, -80, -20)
    for i, a_trip in enumerate(arpha_sols):
        print(f"  [{a_trip[0]:.4f}, {a_trip[1]:.0f}, {a_trip[2]:.0f}]")

    print(f"\n{'=' * 60}")
