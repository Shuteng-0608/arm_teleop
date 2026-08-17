# 右臂离线 IK 轨迹对比

`offline_compare_right_ik.py` 按 CSV 原始帧顺序比较当前
`feasible_ref` 方法和 `redundancy_selector`。程序只调用上位机计算服务，
不会创建关节发布器，也不会调用 `/aris_node` 下位机服务。

## 输入与状态

- 默认输入：`data_log/circle_engineering.csv`；
- 只读取 `right_raw_matrix_*` 和 CSV 原始时间戳；
- 第一帧右手位置作为确定性的映射零点；
- 两种方法使用相同初始关节和初始臂角，后续分别维护状态；
- 下一帧状态按当前 `main_ros` 行为保留 4 位小数；
- 输出 CSV 中成功帧的关节值保留 IK 服务返回的完整精度；
- 求解失败或选择器保持时，仅保持该方法自己的上一有效状态。

默认 `--baseline-angle-source predictor` 会逐帧调用 `/predict_arm_angle`，
并复现 `main_ros` 当前的臂角符号、`0.5` 偏移和搜索顺序。正式记录前必须
重新启动预测服务，确保其历史队列为空。使用
`--baseline-angle-source previous` 可以做不依赖预测器的纯求解器对比。

## 运行边界

运行时只需要：

1. 上位机 ROS master；
2. `/arm_teleop/right_arm_ik_srv`；
3. 使用预测器模式时的 `/predict_arm_angle`。

不需要启动下位机，不要运行 `main_ros.py`、左右臂联合 launch 文件或任何
`/aris_node` 节点。默认处理完整 CSV；`--max-frames` 仅用于正式运行前的
短帧检查。

输出按一帧一行记录源时间戳、目标位姿、预测臂角、两种方法的求解状态、
耗时、七关节值和臂角。速度轨迹应在 Mac 上根据源时间戳做角度解包后差分。
