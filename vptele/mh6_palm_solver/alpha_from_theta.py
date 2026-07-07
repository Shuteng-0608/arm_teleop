"""
给定 theta1, theta2, theta3 (deg)，显式求解 arpha1, arpha2, arpha3 (deg)。

坐标系变量命名（按段序）:
  W_b->W_1: Ry(-61.83241)*Tz(56.9724)*Ry(141.83241)*Rz(arpha2)  -- 变量 arpha2
  W_1->W_2: Ry(-39.03855)*Tz(76.61523)*Ry(-40.96145)*Rz(theta1) -- 变量 theta1
  W_2->W_3: Ry(125.90102)*Tz(67.19535)*Ry(-45.90102)*Rz(theta2) -- 变量 theta2
  W_3->W_4: Ry(114.76245)*Tz(32.21445)*Ry(-79.76245)*Rz(theta3) -- 变量 theta3
  W_4->W_5: Ry(135.16303)*Tz(19.2719)*Ry(-115.16303)*Rz(arpha3) -- 变量 arpha3
  W_5->W_b: Ry(119.75471)*Tz(29.11805)*Ry(105.24529)*Rz(arpha1) -- 变量 arpha1

各段 Ry 净转角:
  R1(arpha2): Ry(80)*Rz(arpha2)
  R2(theta1): Ry(-80)*Rz(theta1)
  R3(theta2): Ry(80)*Rz(theta2)
  R4(theta3): Ry(35)*Rz(theta3)
  R5(arpha3): Ry(20)*Rz(arpha3)
  R6(arpha1): Ry(225)*Rz(arpha1)

约束: R1*R2*R3*R4*R5*R6 = I
  -> K = R2*R3*R4, M = R5*R6, K*M = R1^T

核心推导:
  Element (2,2): k2*m2 = cos80
  -> A*sin(arpha3) + B*cos(arpha3) = C
  -> arpha3 = delta +/- arccos(C/R)

  已知 arpha3 后:
    arpha2 = atan2(v2, -u2)
    arpha1 由 2x2 线性方程组解出

用法:
  from alpha_from_theta import solve_alpha
  solutions = solve_alpha(t1, t2, t3)
"""
import math
import numpy as np

# ============================================================
# 基础: Ry(phi)*Rz(psi) 旋转矩阵 (3x3)
# ============================================================
def RyRz(phi_deg, psi_deg):
    cp = math.cos(math.radians(phi_deg))
    sp = math.sin(math.radians(phi_deg))
    cq = math.cos(math.radians(psi_deg))
    sq = math.sin(math.radians(psi_deg))
    return np.array([
        [ cp*cq, -cp*sq,  sp],
        [    sq,     cq,   0],
        [-sp*cq,  sp*sq,  cp]
    ])

# 旋转常量
COS80 = math.cos(math.radians(80))
SIN80 = math.sin(math.radians(80))
COS20 = math.cos(math.radians(20))
SIN20 = math.sin(math.radians(20))
C225 = math.cos(math.radians(225))  # -sqrt(2)/2
S225 = math.sin(math.radians(225))  # -sqrt(2)/2

# ============================================================
# 核心: 显式求解 arpha3 (W_4->W_5 Rz)
# ============================================================
# 从旋转约束 K*M = R1^T, 取 (2,2) 元素:
#   k2 * col2(M) = cos80
#
# col2(M) = col2(Ry(20)*Rz(arpha3)*Ry(225)*Rz(arpha1))
#         = Ry(20)*Rz(arpha3)*[S225, 0, C225]^T    (arpha1 不影响 col2)
#
# 展开得:
#   A*sin(arpha3) + B*cos(arpha3) = C
#
# 其中:
#   A = k21 * S225
#   B = k20*COS20*S225 - k22*SIN20*S225
#   C = cos80 - k20*SIN20*C225 - k22*COS20*C225

def compute_alpha3_explicit(K):
    """
    从 K = R2*R3*R4 显式求解 arpha3。
    返回 [(arpha3_deg, 类型), ...]。
    """
    k2 = K[2, :]  # K 的第三行

    A = k2[1] * S225
    B = k2[0] * COS20 * S225 - k2[2] * SIN20 * S225
    D = k2[0] * SIN20 * C225 + k2[2] * COS20 * C225
    C = COS80 - D

    R = math.hypot(A, B)

    if R < 1e-15:
        return []  # 退化情况

    ratio = C / R
    if ratio > 1.0:
        if ratio < 1.0 + 1e-12:
            ratio = 1.0
        else:
            return []
    elif ratio < -1.0:
        if ratio > -1.0 - 1e-12:
            ratio = -1.0
        else:
            return []

    delta = math.atan2(A, B)
    phi = math.acos(ratio)

    a3_plus = delta + phi
    a3_minus = delta - phi

    results = []
    for a3 in [a3_minus, a3_plus]:
        a3_deg = math.degrees(a3)
        a3_mod = a3_deg % 360
        if not any(abs(r % 360 - a3_mod) < 1e-10 for r, _ in results):
            results.append((a3_deg, 'explicit' if abs(phi) > 1e-12 else 'double_root'))

    return results


# ============================================================
# 已知 arpha3 后, 显式求解 arpha2 和 arpha1
# ============================================================
def compute_alpha2_and_alpha1(K, a3_deg, t1_deg, t2_deg, t3_deg):
    """
    已知 K 和 arpha3, 求解 arpha2, arpha1。
    返回 (arpha2_deg, arpha1_deg, 旋转误差)。
    """
    ca3 = math.cos(math.radians(a3_deg))
    sa3 = math.sin(math.radians(a3_deg))

    k0 = K[0, :]
    k1 = K[1, :]

    # ---- arpha2 = atan2(v2, -u2) ----
    rz_col = np.array([S225 * ca3, S225 * sa3, C225])
    m_col2 = np.array([
        COS20 * rz_col[0] + SIN20 * rz_col[2],
        rz_col[1],
        -SIN20 * rz_col[0] + COS20 * rz_col[2]
    ])
    u2 = np.dot(k0, m_col2)
    v2 = np.dot(k1, m_col2)

    a2_val = math.degrees(math.atan2(v2, -u2))
    a2_val = (a2_val + 180) % 360 - 180

    # ---- arpha1 由 2x2 线性系统 ----
    rz_w1 = np.array([-sa3, ca3, 0])
    m_w1 = np.array([
        COS20 * rz_w1[0] + SIN20 * rz_w1[2],
        rz_w1[1],
        -SIN20 * rz_w1[0] + COS20 * rz_w1[2]
    ])
    rz_ws = np.array([-C225 * ca3, -C225 * sa3, S225])
    m_ws = np.array([
        COS20 * rz_ws[0] + SIN20 * rz_ws[2],
        rz_ws[1],
        -SIN20 * rz_ws[0] + COS20 * rz_ws[2]
    ])

    v0 = np.dot(k0, m_w1)
    v1 = np.dot(k0, m_ws)
    u0 = np.dot(k1, m_w1)
    u1 = np.dot(k1, m_ws)

    s1 = math.sin(math.radians(a2_val))
    c1 = math.cos(math.radians(a2_val))

    det = v0 * u1 - v1 * u0
    if abs(det) < 1e-15:
        return None

    cos_a1 = (s1 * u1 - c1 * v1) / det
    sin_a1 = (v0 * c1 - u0 * s1) / det
    cos_a1 = max(-1.0, min(1.0, cos_a1))
    sin_a1 = max(-1.0, min(1.0, sin_a1))

    a1_val = math.degrees(math.atan2(sin_a1, cos_a1))
    a1_val = (a1_val + 180) % 360 - 180

    # ---- 验证旋转约束 ----
    Rloop = (RyRz(80, a2_val) @ RyRz(-80, t1_deg) @ RyRz(80, t2_deg) @
             RyRz(35, t3_deg) @ RyRz(20, a3_deg) @ RyRz(225, a1_val))
    err = np.max(np.abs(Rloop - np.eye(3)))

    return a2_val, a1_val, err


# ============================================================
# 主求解接口
# ============================================================
# theta1 限位检查: 物理范围 [270, 360)
# theta2 限位检查: 物理范围 [-90, 0]
# ============================================================
def check_theta1_range(theta1_deg, verbose=False):
    """
    检查 theta1 是否在物理限位 [270, 360) 范围内。
    返回 True/False。
    """
    t1_norm = theta1_deg % 360
    in_range = 270.0 <= t1_norm < 360.0
    if not in_range and verbose:
        print(f"  警告: theta1 = {theta1_deg:.4f} deg (归一化 {t1_norm:.4f} deg) 超出物理限位 [270, 360)")
    return in_range


def check_theta2_range(theta2_deg, verbose=False):
    """
    检查 theta2 是否在物理限位 [-90, 0] 范围内。
    返回 True/False。
    """
    t2_norm = theta2_deg % 360
    if t2_norm > 270:
        t2_norm -= 360
    elif t2_norm > 180:
        t2_norm -= 360
    elif t2_norm < -180:
        t2_norm += 360

    in_range = -90.0 <= t2_norm <= 0.0
    if not in_range and verbose:
        print(f"  警告: theta2 = {theta2_deg:.4f} deg (归一化 {t2_norm:.4f} deg) 超出物理限位 [-90, 0]")
    return in_range


# ============================================================
def solve_alpha(theta1_deg, theta2_deg, theta3_deg, verbose=False):
    """
    给定 theta1, theta2, theta3 (deg)，显式求解 arpha1, arpha2, arpha3 (deg)。

    返回 [(arpha2, arpha3, arpha1, 旋转误差), ...] 按误差排序。
    对应关系: arpha2=W_b->W_1 Rz, arpha3=W_4->W_5 Rz, arpha1=W_5->W_b Rz。
    若无解返回 []。
    """
    t1, t2, t3 = theta1_deg, theta2_deg, theta3_deg

    # 限位检查
    if verbose:
        check_theta1_range(t1, verbose=True)
        check_theta2_range(t2, verbose=True)

    # ---- 计算 K = R2*R3*R4 ----
    K = RyRz(-80, t1) @ RyRz(80, t2) @ RyRz(35, t3)

    # ---- 显式求解 arpha3 ----
    a3_candidates = compute_alpha3_explicit(K)
    if not a3_candidates:
        if verbose:
            print("  无 arpha3 解: |C/R| > 1, theta 值不合法")
        return []

    # ---- 对每个 arpha3 求解 arpha2, arpha1 ----
    results = []
    for a3_val, _ in a3_candidates:
        result = compute_alpha2_and_alpha1(K, a3_val, t1, t2, t3)
        if result is None:
            continue
        a2_val, a1_val, err = result

        results.append((
            a2_val % 360,
            a3_val % 360,
            a1_val % 360,
            err
        ))

    results.sort(key=lambda x: x[3])

    unique = []
    for r in results:
        if not any(abs(r[0]-u[0])<1e-6 and abs(r[1]-u[1])<1e-6 and abs(r[2]-u[2])<1e-6 for u in unique):
            unique.append(r)
    results = unique

    if verbose:
        for i, (a2v, a3v, a1v, err) in enumerate(results):
            print(f"  解 #{i}: arpha2={a2v:.6f} deg, arpha3={a3v:.6f} deg, arpha1={a1v:.6f} deg, err={err:.2e}")

    return results


# ============================================================
# 平移约束
# ============================================================
def p_from_RyTz(phi_deg, d):
    """Ry(phi)*Tz(d) 的平移向量"""
    sa = math.sin(math.radians(phi_deg))
    ca = math.cos(math.radians(phi_deg))
    return np.array([d * sa, 0, d * ca])

# 新坐标系平移参数 (2026-07-03 更新)
P1 = p_from_RyTz(-61.83241, 56.9724)   # W_b->W_1: Ry(-61.83241)*Tz(56.9724)
P2 = p_from_RyTz(-39.03855, 76.61523)  # W_1->W_2: Ry(-39.03855)*Tz(76.61523)
P3 = p_from_RyTz(125.90102, 67.19535)  # W_2->W_3: Ry(125.90102)*Tz(67.19535)
P4 = p_from_RyTz(114.76245, 32.21445)  # W_3->W_4: Ry(114.76245)*Tz(32.21445)
P5 = p_from_RyTz(135.16303, 19.2719)   # W_4->W_5: Ry(135.16303)*Tz(19.2719)
P6 = p_from_RyTz(119.75471, 29.11805)  # W_5->W_b: Ry(119.75471)*Tz(29.11805)


def compute_translation_error(a2_deg, a3_deg, a1_deg, t1_deg, t2_deg, t3_deg):
    """计算平移约束误差 |p_total|"""
    R1 = RyRz(80, a2_deg); R2 = RyRz(-80, t1_deg); R3 = RyRz(80, t2_deg)
    R4 = RyRz(35, t3_deg); R5 = RyRz(20, a3_deg); R6 = RyRz(225, a1_deg)

    R12 = R1 @ R2; R123 = R12 @ R3; R1234 = R123 @ R4; R12345 = R1234 @ R5
    p_total = P1 + R1 @ P2 + R12 @ P3 + R123 @ P4 + R1234 @ P5 + R12345 @ P6
    return np.linalg.norm(p_total)


def solve_alpha_full(theta1_deg, theta2_deg, theta3_deg, verbose=True):
    """
    完整求解：解旋转约束得所有 arpha 候选，选择平移误差最小的。
    返回 (arpha2, arpha3, arpha1, translation_error)
    若无旋转解返回 None。
    """
    if verbose:
        check_theta1_range(theta1_deg, verbose=True)
        check_theta2_range(theta2_deg, verbose=True)
    solutions = solve_alpha(theta1_deg, theta2_deg, theta3_deg, verbose=verbose)
    if not solutions:
        if verbose:
            print("  无旋转解 - 给定的 theta 不在合法配置空间中。")
        return None

    best = None
    for a2v, a3v, a1v, _ in solutions:
        err = compute_translation_error(a2v, a3v, a1v, theta1_deg, theta2_deg, theta3_deg)
        if best is None or err < best[3]:
            best = (a2v, a3v, a1v, err)

    if verbose and best is not None:
        print(f"  最小平移误差 |p_total| = {best[3]:.4e}")
        if best[3] < 1.5:
            print(f"  平移误差 {best[3]:.4e} mm, 在工程容差 (<1.5mm) 内。")
        else:
            print(f"  注意: 平移误差 {best[3]:.4e} mm 超出容差。")

    return best


# ============================================================
# 直接使用全部6个自由度求解 (找任意合法配置)
# ============================================================
def find_valid_config(verbose=True):
    """
    搜索一组合法的 (arpha2,theta1,theta2,theta3,arpha3,arpha1) 配置。
    返回 6 个角度值。
    """
    from scipy.optimize import least_squares

    def residuals(x):
        a2, t1, t2, t3, a3, a1 = x
        R = (RyRz(80, a2) @ RyRz(-80, t1) @ RyRz(80, t2) @
             RyRz(35, t3) @ RyRz(20, a3) @ RyRz(225, a1))
        r_err = (R - np.eye(3)).flatten()

        R1 = RyRz(80, a2); R2 = RyRz(-80, t1); R3 = RyRz(80, t2)
        R4 = RyRz(35, t3); R5 = RyRz(20, a3); R6 = RyRz(225, a1)
        R12 = R1@R2; R123 = R12@R3; R1234 = R123@R4; R12345 = R1234@R5
        p_total = P1 + R1@P2 + R12@P3 + R123@P4 + R1234@P5 + R12345@P6

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

    a2, t1, t2, t3, a3, a1 = best_x
    a2 = (a2 + 180) % 360 - 180
    t1 = (t1 + 180) % 360 - 180
    t2 = (t2 + 180) % 360 - 180
    t3 = (t3 + 180) % 360 - 180
    a3 = (a3 + 180) % 360 - 180
    a1 = (a1 + 180) % 360 - 180

    if verbose:
        print(f"找到合法配置:")
        print(f"  arpha2={a2:.4f} deg, theta1={t1:.4f} deg, theta2={t2:.4f} deg, theta3={t3:.4f} deg, arpha3={a3:.4f} deg, arpha1={a1:.4f} deg")
        print(f"  约束误差: {best_err:.2e}")

    return a2, t1, t2, t3, a3, a1


# ============================================================
# 测试
# ============================================================
if __name__ == "__main__":
    print("=" * 60)
    print("显式求解测试 (新坐标系参数 2026-07-03)")
    print("=" * 60)

    # -------------------------------------------------------
    # 测试 1: 零角度
    # -------------------------------------------------------
    print(f"\n测试 1: 零角度 theta=(0, 0, 0)")
    print("-" * 40)
    sols = solve_alpha(0, 0, 0, verbose=True)
    a2z, a3z, a1z, _ = sols[0]
    pz = compute_translation_error(a2z, a3z, a1z, 0, 0, 0)
    print(f"  平移误差 |p| = {pz:.2e}")
    print()

    # -------------------------------------------------------
    # 测试 2: 非零合法配置
    # -------------------------------------------------------
    print(f"测试 2: 非零合法配置")
    print("-" * 40)
    t1_v, t2_v, t3_v = 359.99969445, 278.88576249, 359.65089575
    a2_v, a3_v, a1_v = 81.19839253, 0.33031357, 0.07147000
    t1_n = (t1_v + 180) % 360 - 180
    t2_n = (t2_v % 360) - 360 if t2_v % 360 > 270 else (t2_v % 360)
    t3_n = (t3_v + 180) % 360 - 180

    print(f"  给定 theta = ({t1_n:.4f}, {t2_n:.4f}, {t3_n:.4f})")
    print(f"  (theta2 = {t2_n:.4f} deg 在物理限位 [-90, 0] 内)")
    print(f"  预期 arpha = ({a2_v:.4f}, {a3_v:.4f}, {a1_v:.4f})")
    print()

    sols2 = solve_alpha(t1_v, t2_v, t3_v, verbose=True)
    print()
    for i, (a2s, a3s, a1s, err) in enumerate(sols2):
        pt = compute_translation_error(a2s, a3s, a1s, t1_v, t2_v, t3_v)
        d = min(abs(a2s - a2_v), abs(a2s - a2_v - 360), abs(a2s - a2_v + 360))
        tag = "匹配预期" if d < 1 else "另一分支"
        print(f"  解 #{i}: arpha2={a2s:.4f}, arpha3={a3s:.4f}, arpha1={a1s:.4f}  |p|={pt:.2e}  ({tag})")
    print()

    # -------------------------------------------------------
    # 测试 3: 不合法 theta
    # -------------------------------------------------------
    print(f"测试 3: 不合法 theta=(30, -45, 60)")
    print("-" * 40)
    sols = solve_alpha(30, -45, 60, verbose=True)
    if not sols:
        print("  无解 (|C/R|>1), 此 theta 不在合法配置空间中。")

    print(f"\n" + "=" * 60)
    print("接口说明:")
    print("  solve_alpha(t1, t2, t3)           -> [(arpha2, arpha3, arpha1, err), ...]")
    print("  solve_alpha_full(t1, t2, t3)      -> (arpha2, arpha3, arpha1, trans_err) 或 None")
    print("  compute_translation_error(a2,a3,a1,t1,t2,t3) -> float")
    print("=" * 60)
