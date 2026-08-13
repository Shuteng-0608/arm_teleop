# 双手轨迹 YAML 到 CSV 工具

本工具用于人工编写双手关键点轨迹，通过三种插值方法生成固定采样序列，并直接
输出已有离线遥操系统可读取的 CSV。它不模拟 Vision Pro，不提供 ROS 节点，
也不包含 gRPC、WebRTC、IK 或机器人下发功能。

## 输出格式

每一行包含 33 列：

1. `timestamp`：从 `0.0` 开始的相对时间，单位为秒。
2. 一只手的 4×4 齐次变换矩阵，共 16 列，按行展开。
3. 另一只手的 4×4 齐次变换矩阵，共 16 列，按行展开。

默认按照本次需求输出左手在前、右手在后：

```text
timestamp,
left_raw_matrix_00,...,left_raw_matrix_33,
right_raw_matrix_00,...,right_raw_matrix_33
```

参考文件 `vp_raw_record_20260715_204843.csv` 的实际顺序与此相反，是右手在前、
左手在后。现有离线读取器通过列名读取，不依赖物理列序，因此两种顺序都能读取。
需要让表头顺序也与历史参考文件完全一致时，编译时增加：

```bash
--column-order right-left
```

这里的 4×4 矩阵是包含旋转和平移的齐次变换矩阵，不只是 3×3 旋转矩阵。矩阵
最后一行为 `[0, 0, 0, 1]`。

## 安装依赖

```bash
cd /path/to/trajectory_csv
python3 -m pip install -r requirements.txt
```

只依赖 NumPy 和 PyYAML，不依赖 ROS。

## 生成 CSV

默认左手在前、右手在后：

```bash
python3 scripts/compile_trajectory.py \
  trajectories/right_square_printer_lookahead.yaml \
  --output outputs/right_square_printer_lookahead.csv
```

与历史参考文件完全相同的右手在前、左手在后顺序：

```bash
python3 scripts/compile_trajectory.py \
  trajectories/right_square_printer_lookahead.yaml \
  --output outputs/right_square_printer_lookahead.csv \
  --column-order right-left
```

程序只负责生成 CSV。生成后的播放和机器人执行继续使用现有系统，不属于本工具
的职责范围。

## YAML 结构

顶层字段：

- `api_version`：固定为 `trajectory_csv/v1`。
- `name`：轨迹名称。
- `trajectory.sample_rate_hz`：CSV 的固定采样频率。
- `trajectory.coordinate_frame`：固定为 `robot_base_delta`。
- `trajectory.interpolation`：插值模式和速度、加速度参数。
- `left_hand`、`right_hand`：左右手轨迹。
- 可选 `csv.initial_wrist_position_m`：覆盖 CSV 首帧双手平移锚点。

YAML 中的 `position_delta_m` 是相对于首帧的机器人末端位移，坐标系为机器人基
坐标系，单位为米。它不是 CSV 矩阵中的绝对平移值。工具会按照现有遥操映射的
逆变换，生成离线 `VPStreamer` 所需的手腕矩阵。

默认的双手首帧位置来自参考 CSV 的第一帧。需要针对另一套录制基准覆盖时，可在
YAML 中加入：

```yaml
csv:
  initial_wrist_position_m:
    left: [-0.165105, 0.203738, 0.828354]
    right: [0.201794, 0.139552, 0.833880]
```

### 单手保持不动

```yaml
left_hand:
  mode: hold_initial
```

### 手工关键点

```yaml
right_hand:
  mode: waypoints
  orientation:
    mode: fixed_initial
  waypoints:
    - name: home
      position_delta_m: [0.00, 0.00, 0.00]
      stop: true
      hold_s: 1.0
    - name: raised
      position_delta_m: [0.00, 0.00, 0.08]
      stop: true
```

关键点字段：

- `position_delta_m: [dx, dy, dz]`：机器人基坐标系中的相对位置。
- `rotation_delta_rotvec_deg: [rx, ry, rz]`：可选的轴角旋转向量，单位为度。
- `stop: true`：在支持的插值模式中要求该点速度为零。
- `hold_s`：到点后停留时间，单位为秒。
- `name`：用于阅读的名称，不影响轨迹计算。

如果第一个关键点不是零位姿，编译器会自动在轨迹前补一个零位姿首帧。

## 三种插值方式

### `stop_and_go`

每段使用五次平滑曲线，从零速度开始并在下一个关键点降到零速度。每个关键点
都是完整停止点，`hold_s` 可增加停留时间。

### `printer_lookahead`

使用类似 3D 打印机的前瞻速度规划，在速度和加速度限制内尽量连续通过普通拐点。
`square_corner_speed_m_s` 控制方形拐角的通过速度；`stop: true` 或 `hold_s`
会把该点变成零速度边界。

### `uniform_linear`

每段按照目标速度做均匀线性采样。普通拐角不减速，速度方向会瞬时改变；
`stop` 标记不参与速度规划，但 `hold_s` 仍会插入停留帧。

三种方形示例使用完全相同的关键点，便于比较插值结果：

```text
trajectories/right_square_stop_and_go.yaml
trajectories/right_square_printer_lookahead.yaml
trajectories/right_square_uniform_linear.yaml
```

## 圆形轨迹

圆形无需手工写大量离散点。前一个普通关键点确定圆的起点和半径，后一个关键点
使用 `circle`：

```yaml
- name: circle_start
  position_delta_m: [0.00, -0.03, 0.11]
- name: circle_complete
  circle:
    center_delta_m: [0.00, 0.00, 0.11]
    normal: [1.00, 0.00, 0.00]
    turns: 1.0
    direction: counterclockwise
  stop: true
```

- `center_delta_m`：圆心，使用同一机器人基坐标系。
- `normal`：圆平面的法向量，不能为零。
- `turns`：圈数，必须大于零。
- `direction`：沿法向观察时的 `counterclockwise` 或 `clockwise`。

编译器按解析圆弧和弧长采样，不需要先用多边形近似圆。

## 验证

```bash
PYTHONPATH=src python3 -m pytest -q test
```

测试覆盖三种插值、圆形轨迹、CSV 33 列契约、矩阵按行展开、齐次矩阵合法性，
以及机器人相对位姿到离线手腕矩阵的逆映射。
