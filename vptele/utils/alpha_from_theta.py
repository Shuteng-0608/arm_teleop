"""
给定 theta₁, theta₂, theta₃ (度)，显式求解 a1, a2, a3 (度)。

核心推导:
  K(theta₁,theta₂,theta₃) · M(a2,a3) = R₁^T(a1)      (旋转约束)
  其中 K = R₂·R₃·R₄,  M = R₅·R₆

  Element (2,2):  k₂·m₂ = cos80
  →  A·sina2 + B·cosa2 = C                (线性三角方程)
  →  a2 = δ ± arccos(C/R)                 (显式闭式解!)

  已知 a2 后:
    a1 = atan2(v₂, -u₂)                    (显式)
    a3 由 2×2 线性方程组解出                (显式)

用法:
  from alpha_from_theta import solve_alpha
  solutions = solve_alpha(t1, t2, t3)      # 返回所有解
"""
import math
import numpy as np
import time


class MH6PalmSolver:

    # ============================================================
    # 基础: Ry(φ)·Rz(ψ) 旋转矩阵 (3x3)
    # ============================================================

    def __init__(self):
        # 常量
        self.COS80 = math.cos(math.radians(80))
        self.SIN80 = math.sin(math.radians(80))
        self.COS20 = math.cos(math.radians(20))
        self.SIN20 = math.sin(math.radians(20))
        self.C225 = math.cos(math.radians(225))  # -√2/2
        self.S225 = math.sin(math.radians(225))  # -√2/2
        pass

    def RyRz(self, phi_deg, psi_deg):
        cp = math.cos(math.radians(phi_deg))
        sp = math.sin(math.radians(phi_deg))
        cq = math.cos(math.radians(psi_deg))
        sq = math.sin(math.radians(psi_deg))
        return np.array([
            [ cp*cq, -cp*sq,  sp],
            [    sq,     cq,   0],
            [-sp*cq,  sp*sq,  cp]
        ])

    

    # ============================================================
    # 核心: 显式求解 a2
    # ============================================================
    # 从旋转约束 K·M = R₁^T, 取 (2,2) 元素:
    #   k₂ · col₂(M) = cos80
    #
    # col₂(M) = col₂(Ry(20)·Rz(a2)·Ry(225)·Rz(a3))
    #         = Ry(20) · Rz(a2) · [S225, 0, C225]^T    (a3 不影响 col₂)
    #
    # 展开得:
    #   A·sina2 + B·cosa2 = C
    #
    # 其中:
    #   A = k₂₁ · S225
    #   B = k₂₀·COS20·S225 - k₂₂·SIN20·S225
    #   C = cos80 - k₂₀·SIN20·C225 - k₂₂·COS20·C225

    def compute_alpha2_explicit(self, K):
        """
        从 K = R₂·R₃·R₄ 显式求解 a2。
        返回 [(a2_deg, 类型), ...] 类型说明解的性质。
        """
        k2 = K[2, :]  # K 的第三行

        A = k2[1] * self.S225
        B = k2[0] * self.COS20 * self.S225 - k2[2] * self.SIN20 * self.S225
        D = k2[0] * self.SIN20 * self.C225 + k2[2] * self.COS20 * self.C225
        C = self.COS80 - D

        R = math.hypot(A, B)

        # 检查是否可解
        if R < 1e-15:
            return []  # 退化情况

        ratio = C / R
        if ratio > 1.0:
            if ratio < 1.0 + 1e-12:
                ratio = 1.0
            else:
                return []  # |C/R| > 1, 无解
        elif ratio < -1.0:
            if ratio > -1.0 - 1e-12:
                ratio = -1.0
            else:
                return []

        delta = math.atan2(A, B)  # a2 的"基础偏移"
        phi = math.acos(ratio)

        a2_plus = delta + phi
        a2_minus = delta - phi

        # 去重 (两个解可能重合, 当 phi=0 即 |C/R|=1)
        results = []
        for a2 in [a2_minus, a2_plus]:
            a2_deg = math.degrees(a2)
            a2_mod = a2_deg % 360
            if not any(abs(r % 360 - a2_mod) < 1e-10 for r, _ in results):
                results.append((a2_deg, 'explicit' if abs(phi) > 1e-12 else 'double_root'))

        return results


    # ============================================================
    # 已知 a2 后, 显式求解 a1 和 a3
    # ============================================================
    def compute_alpha1_and_alpha3(self, K, a2_deg, t1_deg, t2_deg, t3_deg):
        """
        已知 K 和 a2, 求解 a1, a3。
        返回 (a1_deg, a3_deg, 旋转误差)。
        """
        ca2 = math.cos(math.radians(a2_deg))
        sa2 = math.sin(math.radians(a2_deg))

        k0 = K[0, :]
        k1 = K[1, :]

        # ---- a1 = atan2(v₂, −u₂) ----
        rz_col = np.array([self.S225 * ca2, self.S225 * sa2, self.C225])
        m_col2 = np.array([
            self.COS20 * rz_col[0] + self.SIN20 * rz_col[2],
            rz_col[1],
            -self.SIN20 * rz_col[0] + self.COS20 * rz_col[2]
        ])
        u2 = np.dot(k0, m_col2)
        v2 = np.dot(k1, m_col2)

        a1_val = math.degrees(math.atan2(v2, -u2))
        a1_val = (a1_val + 180) % 360 - 180  # 标准化到 [-180, 180]

        # ---- a3 由 2×2 线性系统 ----
        rz_w1 = np.array([-sa2, ca2, 0])
        m_w1 = np.array([
            self.COS20 * rz_w1[0] + self.SIN20 * rz_w1[2],
            rz_w1[1],
            -self.SIN20 * rz_w1[0] + self.COS20 * rz_w1[2]
        ])
        rz_ws = np.array([-self.C225 * ca2, -self.C225 * sa2, self.S225])
        m_ws = np.array([
            self.COS20 * rz_ws[0] + self.SIN20 * rz_ws[2],
            rz_ws[1],
            -self.SIN20 * rz_ws[0] + self.COS20 * rz_ws[2]
        ])

        # v₀c₃ + v₁s₃ = s₁  (k₀·col₁(M) = s₁)
        # u₀c₃ + u₁s₃ = c₁  (k₁·col₁(M) = c₁)
        v0 = np.dot(k0, m_w1)
        v1 = np.dot(k0, m_ws)
        u0 = np.dot(k1, m_w1)
        u1 = np.dot(k1, m_ws)

        s1 = math.sin(math.radians(a1_val))
        c1 = math.cos(math.radians(a1_val))

        det = v0 * u1 - v1 * u0
        if abs(det) < 1e-15:
            return None  # 退化

        cos_a3 = (s1 * u1 - c1 * v1) / det
        sin_a3 = (v0 * c1 - u0 * s1) / det
        cos_a3 = max(-1.0, min(1.0, cos_a3))
        sin_a3 = max(-1.0, min(1.0, sin_a3))

        a3_val = math.degrees(math.atan2(sin_a3, cos_a3))
        a3_val = (a3_val + 180) % 360 - 180

        # ---- 验证旋转约束 ----
        Rloop = (self.RyRz(80, a1_val) @ self.RyRz(-80, t1_deg) @ self.RyRz(80, t2_deg) @
                self.RyRz(35, t3_deg) @ self.RyRz(20, a2_deg) @ self.RyRz(225, a3_val))
        err = np.max(np.abs(Rloop - np.eye(3)))

        return a1_val, a3_val, err


    # ============================================================
    # 主求解接口
    # ============================================================
    def solve_alpha(self, theta1_deg, theta2_deg, theta3_deg, verbose=False):
        """
        给定 theta₁, theta₂, theta₃ (度)，显式求解 a1, a2, a3 (度)。

        返回列表 [(a1, a2, a3, 旋转误差), ...] 按误差排序。
        若无解返回 []。
        """
        t1, t2, t3 = theta1_deg, theta2_deg, theta3_deg

        # ---- 计算 K = R₂·R₃·R₄ ----
        K = self.RyRz(-80, t1) @ self.RyRz(80, t2) @ self.RyRz(35, t3)

        # ---- 显式求解 a2 ----
        a2_candidates = self.compute_alpha2_explicit(K)
        if not a2_candidates:
            if verbose:
                print("  无 a2 解: |C/R| > 1, theta 值不合法")
            return []

        # ---- 对每个 a2 求解 a1, a3 ----
        results = []
        for a2_val, _ in a2_candidates:
            result = self.compute_alpha1_and_alpha3(K, a2_val, t1, t2, t3)
            if result is None:
                continue
            a1_val, a3_val, err = result

            # 标准化到 [0, 360) 便于比较
            results.append((
                a1_val % 360,
                a2_val % 360,
                a3_val % 360,
                err
            ))

        # 按误差排序
        results.sort(key=lambda x: x[3])

        # 去除重复解
        unique = []
        for r in results:
            if not any(abs(r[0]-u[0])<1e-6 and abs(r[1]-u[1])<1e-6 and abs(r[2]-u[2])<1e-6 for u in unique):
                unique.append(r)
        results = unique

        if verbose:
            for i, (a1v, a2v, a3v, err) in enumerate(results):
                print(f"  解 #{i}: a1={a1v:.6f} deg, a2={a2v:.6f} deg, a3={a3v:.6f} deg, err={err:.2e}")

        return results


    # ============================================================
    # 完整求解器 (含平移约束验证)
    # ============================================================
    def compute_translation_error(self, a1_deg, a2_deg, a3_deg, t1_deg, t2_deg, t3_deg):
        """计算平移约束误差 |p_total|"""
        def p_from_RyTz(a_deg, d):
            sa = math.sin(math.radians(a_deg))
            ca = math.cos(math.radians(a_deg))
            return np.array([d * sa, 0, d * ca])

        p1 = np.array([-51.70241, 0, 26.63347])
        p2 = p_from_RyTz(-38.80423, 77.00424)
        p3 = p_from_RyTz(125.45738, 67.70538)
        p4 = p_from_RyTz(114.32486, 32.10229)
        p5 = p_from_RyTz(122.67927, 17.87902)
        p6 = p_from_RyTz(126.52256, 31.45639)

        R1 = self.RyRz(80, a1_deg); R2 = self.RyRz(-80, t1_deg); R3 = self.RyRz(80, t2_deg)
        R4 = self.RyRz(35, t3_deg); R5 = self.RyRz(20, a2_deg); R6 = self.RyRz(225, a3_deg)

        R12 = R1 @ R2; R123 = R12 @ R3; R1234 = R123 @ R4; R12345 = R1234 @ R5
        p_total = p1 + R1 @ p2 + R12 @ p3 + R123 @ p4 + R1234 @ p5 + R12345 @ p6
        return np.linalg.norm(p_total)


    def solve_alpha_full(self, theta1_deg, theta2_deg, theta3_deg, verbose=True):
        """
        完整求解：解旋转约束得所有 α 候选，选择平移误差最小的。
        返回 (alpha1, alpha2, alpha3, translation_error)
        若无旋转解返回 None。
        """
        solutions = self.solve_alpha(theta1_deg, theta2_deg, theta3_deg, verbose=verbose)
        if not solutions:
            if verbose:
                print("  无旋转解 - 给定的 theta 不在合法配置空间中。")
            return None

        # 对每个解计算平移误差，选最佳
        best = None
        for a1v, a2v, a3v, _ in solutions:
            err = self.compute_translation_error(a1v, a2v, a3v, theta1_deg, theta2_deg, theta3_deg)
            if best is None or err < best[3]:
                best = (a1v, a2v, a3v, err)

        if verbose and best is not None:
            print(f"  最小平移误差 |p_total| = {best[3]:.4e}")
            if best[3] > 1e-3:
                print(f"  注意: 平移约束不满足! 这组 theta 不满足完整闭环条件。")
            else:
                print(f"  旋转和平移约束均满足。")

        return best


    # ============================================================
    # 直接使用全部6个自由度求解 (找任意合法配置)
    # ============================================================
    def find_valid_config(self, verbose=True):
        """
        搜索一组合法的 (a1,theta₁,theta₂,theta₃,a2,a3) 配置。
        返回 6 个角度值。
        """
        from scipy.optimize import least_squares

        def residuals(x):
            a1, t1, t2, t3, a2, a3 = x
            R = (self.RyRz(80, a1) @ self.RyRz(-80, t1) @ self.RyRz(80, t2) @
                self.RyRz(35, t3) @ self.RyRz(20, a2) @ self.RyRz(225, a3))
            r_err = (R - np.eye(3)).flatten()

            p1 = np.array([-51.70241, 0, 26.63347])
            def p_RyTz(a, d):
                sa = math.sin(math.radians(a)); ca = math.cos(math.radians(a))
                return np.array([d*sa, 0, d*ca])
            p2 = p_RyTz(-38.80423, 77.00424)
            p3 = p_RyTz(125.45738, 67.70538)
            p4 = p_RyTz(114.32486, 32.10229)
            p5 = p_RyTz(122.67927, 17.87902)
            p6 = p_RyTz(126.52256, 31.45639)
            R1 = self.RyRz(80, a1); R2 = self.RyRz(-80, t1); R3 = self.RyRz(80, t2)
            R4 = self.RyRz(35, t3); R5 = self.RyRz(20, a2); R6 = self.RyRz(225, a3)
            R12 = R1@R2; R123 = R12@R3; R1234 = R123@R4; R12345 = R1234@R5
            p_total = p1 + R1@p2 + R12@p3 + R123@p4 + R1234@p5 + R12345@p6

            return np.concatenate([r_err, p_total])

        np.random.seed(42)
        best_err = 1e10
        best_x = None
        for _ in range(100):
            guess = np.random.uniform(-180, 180, 6)
            try:
                sol = least_squares(residuals, guess, method='lm', xtol=1e-14, ftol=1e-14, max_nfev=500)
                err = np.max(np.abs(sol.fun))
                if err < best_err:
                    best_err = err
                    best_x = sol.x
                    if err < 1e-10:
                        break
            except Exception:
                pass

        if best_x is None or best_err > 1e-6:
            raise RuntimeError(f"未找到合法配置。最佳误差: {best_err:.2e}")

        a1, t1, t2, t3, a2, a3 = best_x
        a1 = (a1 + 180) % 360 - 180
        t1 = (t1 + 180) % 360 - 180
        t2 = (t2 + 180) % 360 - 180
        t3 = (t3 + 180) % 360 - 180
        a2 = (a2 + 180) % 360 - 180
        a3 = (a3 + 180) % 360 - 180

        if verbose:
            print(f"找到合法配置:")
            print(f"  a1={a1:.4f} deg, theta1={t1:.4f} deg, theta2={t2:.4f} deg, theta3={t3:.4f} deg, a2={a2:.4f} deg, a3={a3:.4f} deg")
            print(f"  约束误差: {best_err:.2e}")

        return a1, t1, t2, t3, a2, a3


# ============================================================
# 测试
# ============================================================
if __name__ == "__main__":

    solver = MH6PalmSolver()
    # 已知合法配置
    a1_true, t1_true, t2_true, t3_true, a2_true, a3_true = \
        87.1050, 120.3487, 251.6664, 11.5087, 136.0891, 209.1478

    print("=" * 60)
    print("显式求解测试")
    print("=" * 60)

    # 测试 1: 已知合法 theta → 恢复 α
    print(f"\n测试 1: theta=({t1_true:.4f}, {t2_true:.4f}, {t3_true:.4f})")
    print("-" * 40)
    time_now = time.time()
    sols = solver.solve_alpha(t1_true, t2_true, t3_true, verbose=True)
    print(f"求解耗时: {time.time() - time_now}")
    print(f"\n找到 {len(sols)} 个旋转解:")
    for i, (a1v, a2v, a3v, err) in enumerate(sols):
        t_err = solver.compute_translation_error(a1v, a2v, a3v, t1_true, t2_true, t3_true)
        print(f"  解 #{i}: a1={a1v:.6f}, a2={a2v:.6f}, a3={a3v:.6f}")
        print(f"          R_err={err:.2e}, |p|={t_err:.4e}")
        if t_err < 1e-3:
            print(f"  [OK] 平移约束满足 (|p|<1e-3)")
        else:
            print(f"  [FAIL] 平移约束不满足")

    # 测试 2: 零角度 (应该满足)
    print(f"\n测试 2: 零角度 theta=(0, 0, 0)")
    print("-" * 40)
    sols = solver.solve_alpha(0, 0, 0, verbose=True)

    # 测试 3: 随机不合法 theta (预期无解)
    print(f"\n测试 3: 随机 theta=(30, -45, 60) (预期可能无解)")
    print("-" * 40)
    sols = solver.solve_alpha(30, -45, 60, verbose=True)
    if not sols:
        print("  无解 (|C/R|>1), 此 theta 不合法。")

    print(f"\n" + "=" * 60)
    print("接口说明:")
    print("  solve_alpha(t1, t2, t3)           → [(a1,a2,a3,err), ...]")
    print("  solve_alpha_full(t1, t2, t3)      → (a1,a2,a3,trans_err) 或 None")
    print("  compute_translation_error(...)     → float")
    print("=" * 60)
