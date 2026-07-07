# 归一化求解器使用说明

## 用法

```python
from solve_from_arpha2_arpha3_theta1 import solve_from_normalized

# 输入：三个 [0,1] 之间的数
u1, u2, u3 = 0.5, 0.5, 0.0

# 输出：六组关节角 + 验证信息
solutions = solve_from_normalized(u1, u2, u3)
# 每条结果为 (theta2, theta3, arpha1, 旋转误差, 平移误差|p|)
```

## 归一化映射

| 用户输入 u ∈ [0,1] | 物理角度 | 限位范围 |
|:-:|:-:|:-:|
| u₁ → arpha2 | arpha2 = 90 × u₁ | [0°, 90°] |
| u₂ → arpha3 | arpha3 = 180 × u₂ | [0°, 180°] |
| u₃ → theta1 | theta1 = −40 × u₃ | [−40°, 0°] |

### 示例

| (u₁, u₂, u₃) | (arpha2, arpha3, theta1) |
|:---:|:---:|
| (0, 0, 0) | (0°, 0°, 0°) |
| (1, 1, 1) | (90°, 180°, −40°) |
| (0.5, 0.5, 0) | (45°, 90°, 0°) |

## 输出说明

`solve_from_normalized` 返回 `[(theta2, theta3, arpha1, rot_err, trans_err), ...]`

| 输出 | 含义 |
|:---:|:---|
| theta2 | W₂→W₃ 的 Rz 转角（关节角） |
| theta3 | W₃→W₄ 的 Rz 转角（关节角） |
| arpha1 | W₅→Wb 的 Rz 转角（关节角） |
| rot_err | 旋转闭合误差（~1e-15 为精确闭合） |
| trans_err | 平移闭合误差 \|p\|，单位 mm（~6e-6 为精确闭合） |

可能会返回多个解（不同分支），通常第一个解误差最小。

## 物理限位总结

| 参数 | 限位范围 | 说明 |
|:---:|:---:|:---|
| arpha2 | [0°, 90°] | 输入限位 |
| arpha3 | [0°, 180°] | 输入限位 |
| theta1 | [−40°, 0°] | 输入限位 |
| theta2 | [−90°, 0°] | 输出限位（需求解后自行检查） |

## 文件结构

```
alpha1_theta1_alpha3已知求解器封装包/
├── solve_from_arpha2_arpha3_theta1.py    ← 主求解器（归一化输入 + 角度输入）
├── solve_from_alpha1.py                   ← 旧求解器（arpha2,theta1,arpha1 → theta2,theta3,arpha3）
├── alpha_from_theta.py                    ← 正向求解器（theta1,theta2,theta3 → arpha2,arpha3,arpha1）
├── README.md                              ← 本文件
└── 使用说明.txt
```

## 依赖

- Python ≥ 3.8
- numpy
