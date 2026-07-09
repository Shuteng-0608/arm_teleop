# MH6PalmSolver — st_7_7 版本

## 快速使用

三个归一化值 [0,1]³ → 三组电机值：

```python
from solve_from_arpha2_arpha3_theta1 import MH6PalmSolver

solver = MH6PalmSolver()

motors = solver.solve_motor_from_normalized(0.641, 0.582, 0.885)
# → [[420.6, 303.1, 582.1], [480.6, 303.1, 582.1]]
#     [m1,    m2,    m3  ]   每个解一个三元组
```

## SolidWorks 验证

输入 `(arpha2, arpha3, theta1) = (47°, -80°, -20°)`：

| 参数 | SolidWorks 实测 | 求解器输出 | 误差 |
|:---:|:---:|:---:|:---:|
| theta2 | -54.2312° | -54.2312° | < 1e-4° |
| theta3 | 75.66174° | 75.6617° | < 1e-4° |
| arpha1 | -4.4° | -4.3990° | 0.001° |

旋转闭合误差 `rot_err ≈ 2e-16`，平移闭合误差 `|p| ≈ 5e-6 mm`。

## 用法

```python
from solve_from_arpha2_arpha3_theta1 import MH6PalmSolver

solver = MH6PalmSolver()

# 角度输入 -> arpha 三元组
solutions = solver.solve_arpha(47, -80, -20)
# → [[-4.3990, -47, -80], [-19.0601, -47, -80]]

# 归一化输入 -> arpha 三元组
solutions = solver.solve_arpha_from_normalized(0.641, 0.582, 0.885)
# → [[-4.6139, -47, -80], ...]

# 电机值输出
motors = solver.solve_motor(47, -80, -20)
# → [[m1, m2, m3], ...]
```

## 归一化映射

| u ∈ [0,1] | 物理角度 | 范围 |
|:---:|:---:|:---:|
| u₁ → arpha2 | 121.9·u₁ − 31.1 | [-31.1°, 90.8°] |
| u₂ → arpha3 | −239·u₂ + 59 | [-180°, 59°] |
| u₃ → theta1 | −32.3·u₃ + 8.6 | [-23.7°, 8.6°] |

## 电机标定

| 关节 | 公式 | 校准点 |
|:---:|:---|:---:|
| arpha1 | 500 + (99.0/23.6)·arpha1 | 0°→500, −23.6°→401 |
| arpha2 | 500 − (380.0/90.8)·arpha2 | 0°→500, 90.8°→120 |
| arpha3 | 247 − (753.0/180.0)·arpha3 | 0°→247, −180°→1000 |

## 输出约定

`arpha2* = -arpha2`（输出时取负，输入用原始值）
