"""
给定 arpha2, theta1, arpha1 (deg)，显式求解 theta2, theta3, arpha3 (deg)。

变量命名：
  arpha2 (已知) - Wb->W1 的 Rz 转角 (R1 = Ry(80)*Rz(arpha2))
  theta1 (已知) - W1->W2 的 Rz 转角 (R2 = Ry(-80)*Rz(theta1))
  arpha1 (已知) - W5->Wb 的 Rz 转角 (R6 = Ry(225)*Rz(arpha1))
  theta2 (待求) - W2->W3 的 Rz 转角 (R3 = Ry(80)*Rz(theta2))
  theta3 (待求) - W3->W4 的 Rz 转角 (R4 = Ry(35)*Rz(theta3))
  arpha3 (待求) - W4->W5 的 Rz 转角 (R5 = Ry(20)*Rz(arpha3))

核心推导:
  约束 R1*R2*R3*R4*R5*R6 = I
  -> R3*R4*R5 = (R1*R2)^T * R6^T = Q

  取 D = Ry(80)^T * Q = Ry(-80) * Q
  -> Rz(theta2)*Ry(35)*Rz(theta3)*Ry(20)*Rz(arpha3) = D

  Element (3,3): D[2,2] = [0,0,1]*Ry(35)*Rz(theta3)*Ry(20)*[0,0,1]^T
  = -sin35*sin20*cos(theta3) + cos35*cos20
  -> cos(theta3) = (cos35*cos20 - D[2,2]) / (sin35*sin20)

  已知 theta3 -> theta2 = atan2(E[0,2]*D[1,2] - E[1,2]*D[0,2],
                                E[0,2]*D[0,2] + E[1,2]*D[1,2])
  其中 E = Ry(35)*Rz(theta3)*Ry(20)

  已知 theta2,theta3 -> arpha3 = atan2(J[1,0], J[1,1])
  其中 J = Rz(-theta3)*Ry(-35)*Rz(-theta2)*D

各段 Ry 净转角:
  R1: Ry(80)*Rz(arpha2), R2: Ry(-80)*Rz(theta1), R3: Ry(80)*Rz(theta2)
  R4: Ry(35)*Rz(theta3),  R5: Ry(20)*Rz(arpha3),  R6: Ry(225)*Rz(arpha1)

物理限位:
  theta1 in [270, 360)
  theta2 in [-90, 0]

用法:
  from solve_from_alpha1 import solve_remaining
  solutions = solve_remaining(arpha2, theta1, arpha1)
  # 每条结果为 (theta2, theta3, arpha3, 旋转误差, 平移误差|p|)
"""
import math
import numpy as np

# 常量
C80 = math.cos(math.radians(80))
S80 = math.sin(math.radians(80))
C35 = math.cos(math.radians(35))
S35 = math.sin(math.radians(35))
C20 = math.cos(math.radians(20))
S20 = math.sin(math.radians(20))


def RyRz(phi_deg, psi_deg):
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


# ---- 平移参数 (2026-07-03 更新) ----
def p_from_RyTz(phi_deg, d):
    sa = math.sin(math.radians(phi_deg))
    ca = math.cos(math.radians(phi_deg))
    return np.array([d * sa, 0, d * ca])

P1 = p_from_RyTz(-61.83241, 56.9724)
P2 = p_from_RyTz(-39.03855, 76.61523)
P3 = p_from_RyTz(125.90102, 67.19535)
P4 = p_from_RyTz(114.76245, 32.21445)
P5 = p_from_RyTz(135.16303, 19.2719)
P6 = p_from_RyTz(119.75471, 29.11805)


def compute_translation_error(arpha2_deg, arpha3_deg, arpha1_deg, theta1_deg, theta2_deg, theta3_deg):
    """计算平移约束误差 |p_total|"""
    R1 = RyRz(80, arpha2_deg)
    R2 = RyRz(-80, theta1_deg)
    R3 = RyRz(80, theta2_deg)
    R4 = RyRz(35, theta3_deg)
    R5 = RyRz(20, arpha3_deg)
    R6 = RyRz(225, arpha1_deg)
    R12 = R1 @ R2; R123 = R12 @ R3; R1234 = R123 @ R4; R12345 = R1234 @ R5
    p_total = P1 + R1 @ P2 + R12 @ P3 + R123 @ P4 + R1234 @ P5 + R12345 @ P6
    return np.linalg.norm(p_total)


def check_theta1_range(t1_deg, verbose=False):
    """Check theta1 physical limit [270, 360)"""
    ok = (270 <= t1_deg % 360 < 360) or abs(t1_deg % 360 - 360) < 1e-9
    if verbose:
        t1_mod = t1_deg % 360
        print(f"  theta1 = {t1_mod:.2f} deg -> {'OK' if ok else 'OUT OF RANGE'} (limit [270, 360))")
    return ok


def check_theta2_range(t2_deg, verbose=False):
    """Check theta2 physical limit [-90, 0]"""
    ok = -90 <= t2_deg <= 0
    if verbose:
        print(f"  theta2 = {t2_deg:.2f} deg -> {'OK' if ok else 'OUT OF RANGE'} (limit [-90, 0])")
    return ok


def _norm_to_180(deg):
    return (deg + 180) % 360 - 180


def check_arpha3_range(arpha3_deg, verbose=False):
    """Check arpha3 physical limit [-20, 20]"""
    v = _norm_to_180(arpha3_deg)
    ok = -20 <= v <= 20
    if verbose:
        print(f"  arpha3 = {v:.2f} deg -> {'OK' if ok else 'OUT OF RANGE'} (limit [-20, 20])")
    return ok


def solve_remaining(arpha2_deg, theta1_deg, arpha1_deg):
    """
    Given arpha2, theta1, arpha1, find theta2, theta3, arpha3.

    Returns [(theta2, theta3, arpha3, rotation_error, translation_error), ...]
    sorted by rotation error. Returns [] if no solution exists.
    """
    # ---- theta1 physical limit check ----
    if not check_theta1_range(theta1_deg):
        return []

    # ---- known rotations ----
    R1 = RyRz(80, arpha2_deg)
    R2 = RyRz(-80, theta1_deg)
    R6 = RyRz(225, arpha1_deg)

    # ---- Q = (R1*R2)^T * R6^T ----
    R12 = R1 @ R2
    Q = R12.T @ R6.T

    # ---- D = Ry(-80) * Q ----
    Ry_neg80 = RyRz(-80, 0)
    D = Ry_neg80 @ Q

    # ---- explicit theta3 formula ----
    val = (C35 * C20 - D[2, 2]) / (S35 * S20)
    if abs(val) > 1.0 + 1e-12:
        return []  # no real solution
    val = max(-1.0, min(1.0, val))
    phi = math.acos(val)
    theta3_candidates = [phi, -phi]  # +/- branch

    results = []
    for t3_val in theta3_candidates:
        t3_deg = math.degrees(t3_val)
        t3_mod = t3_deg % 360

        # ---- E = Ry(35)*Rz(theta3)*Ry(20) ----
        E = RyRz(35, t3_mod) @ RyRz(20, 0)

        # ---- theta2 = atan2(...) ----
        a, b = D[0, 2], D[1, 2]
        e02, e12 = E[0, 2], E[1, 2]
        denom = e02 * a + e12 * b
        numer = e02 * b - e12 * a
        t2_val = math.atan2(numer, denom)
        t2_deg = (math.degrees(t2_val) + 180) % 360 - 180

        # ---- arpha3 = atan2(J[1,0], J[1,1]) ----
        ct2, st2 = math.cos(t2_val), math.sin(t2_val)
        Rz_neg_t2 = np.array([
            [ct2, st2, 0],
            [-st2, ct2, 0],
            [0, 0, 1]
        ])
        Ry_neg35 = RyRz(-35, 0)
        ct3, st3 = math.cos(t3_val), math.sin(t3_val)
        Rz_neg_t3 = np.array([
            [ct3, st3, 0],
            [-st3, ct3, 0],
            [0, 0, 1]
        ])

        G = Rz_neg_t2 @ D
        H = Ry_neg35 @ G
        J = Rz_neg_t3 @ H

        a3_val_inner = math.atan2(J[1, 0], J[1, 1])
        a3_deg_inner = (math.degrees(a3_val_inner) + 180) % 360 - 180

        # ---- verify rotation + translation ----
        R3 = RyRz(80, t2_deg)
        R4 = RyRz(35, t3_mod)
        R5 = RyRz(20, a3_deg_inner)
        R345 = R3 @ R4 @ R5
        rot_err = np.max(np.abs(R345 - Q))

        trans_err = compute_translation_error(
            arpha2_deg, a3_deg_inner, arpha1_deg, theta1_deg, t2_deg, t3_mod)

        results.append((t2_deg % 360, t3_mod, a3_deg_inner % 360, rot_err, trans_err))

    # deduplicate + sort by error
    unique = []
    for r in results:
        if not any(abs(r[0]-u[0])<1e-6 and abs(r[1]-u[1])<1e-6 and abs(r[2]-u[2])<1e-6 for u in unique):
            unique.append(r)
    unique.sort(key=lambda x: x[3])
    return unique


# ============================================================
if __name__ == "__main__":
    print("=" * 60)
    print("arpha2,theta1,arpha1 -> theta2,theta3,arpha3 solver test")
    print("=" * 60)

    # Test 1: near-zero (arpha2=0, theta1=270, arpha1=90 -> theta2=0, theta3=0)
    print("\nTest 1: near-zero (arpha2=0, theta1=270, arpha1=90)")
    sols = solve_remaining(0, 270, 90)
    for t2, t3, a3_val, rot_err, trans_err in sols:
        t2d = t2 if t2 <= 180 else t2 - 360
        a3_ok = check_arpha3_range(a3_val)
        print(f"  theta2={t2:.4f} deg [{t2d:.2f}], theta3={t3:.4f} deg, arpha3={a3_val:.4f} deg {'OK' if a3_ok else 'LIMIT'}  rot_err={rot_err:.2e}  |p|={trans_err:.2e}")
    if not sols:
        print("  (no solution)")

    # Test 2: random valid values
    print("\nTest 2: random (arpha2=30, theta1=270, arpha1=90)")
    sols = solve_remaining(30, 270, 90)
    for t2, t3, a3_val, rot_err, trans_err in sols:
        t2d = t2 if t2 <= 180 else t2 - 360
        a3_ok = check_arpha3_range(a3_val)
        print(f"  theta2={t2:.4f} deg [{t2d:.2f}], theta3={t3:.4f} deg, arpha3={a3_val:.4f} deg {'OK' if a3_ok else 'LIMIT'}  rot_err={rot_err:.2e}  |p|={trans_err:.2e}")
    if not sols:
        print("  (no solution)")

    # Test 3: bidirectional verification with original solver
    print("\nTest 3: bidirectional (theta1=270, theta2=-40, theta3=90)")
    from alpha_from_theta import solve_alpha
    sols_fwd = solve_alpha(270, -40, 90)
    if sols_fwd:
        a2_v, a3_v, a1_v, _ = sols_fwd[0]
        print(f"  Forward: arpha2={a2_v:.4f} deg, arpha3={a3_v:.4f} deg, arpha1={a1_v:.4f} deg")
        sols_bwd = solve_remaining(a2_v, 270, a1_v)
        for t2, t3, a3_b, rot_err, trans_err in sols_bwd:
            t2d = t2 if t2 <= 180 else t2 - 360
            a3_ok = check_arpha3_range(a3_b)
            print(f"  Backward: theta2={t2:.4f} deg [{t2d:.2f}], theta3={t3:.4f} deg, arpha3={a3_b:.4f} deg {'OK' if a3_ok else 'LIMIT'}  rot_err={rot_err:.2e}  |p|={trans_err:.2e}")
        if not sols_bwd:
            print("  (no solution)")
    else:
        print("  FAIL: forward solver returned no solution")

    print(f"\n{'=' * 60}")
    print("All tests done")
    print("=" * 60)
