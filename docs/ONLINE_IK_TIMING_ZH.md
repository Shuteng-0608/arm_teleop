# 右臂在线回放：逐帧求解耗时记录

2026-10-02。适用分支 `for_cjp`，A1 与 `minimum_sufficient_continuity_refined` 两种回放方法。

修改前已通过 HTTPS `git fetch origin` 核对：本地与远端均为
`c7b80f0867cbf24e6f7330266758ff4c86e6a698`，工作区干净、领先/落后均为 0。
本次只在 Mac 修改上位机仓库；未连接设备、未推送。没有更改轨迹、初值、求解策略、运动预算、下位机或运动学仓库。

## 使用方式

此次修改了 `ArmIK.srv`，所以不能只复制 Python 脚本。部署后需要在上位机重新生成服务代码和编译 C++ 节点：

```bash
cd /home/pangu/pangu
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

停止旧 IK 服务和相关客户端，在已经 source 新工作空间的终端重新启动原来的服务流程，例如：

```bash
roslaunch arm_teleop teleop_service.launch
```

另一个已 source 的终端运行原回放命令，无需增加计时参数：

```bash
# A1；这是实际运动命令，须按原有现场操作流程执行
rosrun arm_teleop playback_right_teleop.py --ik-method A1_minimum_jv

# 或冻结策略，不要同时运行两次回放
rosrun arm_teleop playback_right_teleop.py --ik-method minimum_sufficient_continuity_refined
```

也可先用 `--ik-only` 检查计时输出；该模式仅调用 IK，不连接下位机、不发布运动指令。
`--preflight` 不调用 IK，因此不产生逐帧求解计时。

`ArmIK` 服务类型的 MD5 会改变。凡使用该服务类型的客户端和服务端都必须使用本次生成的消息并重启，包括同一 launch 中的左臂服务。左臂及旧 feasible/optimal 方法不提供此处的模块诊断，其 `timing_valid` 保持 false；不能把默认零值当作真实测量。下位机服务、话题消息定义未改变。

## 输出文件

设原回放结果为 `data_log/online_right_teleop_时间.csv`，会在同目录生成：

| 文件 | 内容 |
| --- | --- |
| 原 `.csv` | 原有轨迹数据，追加服务端计时列 |
| `_timing.csv` | 每一次 IK 调用的独立计时、帧号、源时间、方法、状态 |
| `_timing_summary.csv` | 可直接打开的模块统计表 |
| `_timing_summary.json` | 同样的统计及统计口径说明 |

指定 `--output /path/run.csv` 时，附加文件跟随该路径和文件名前缀。禁止覆盖已有文件。
正常实机回放的 audit JSON 也会写入计时文件路径。

独立计时记录在收到响应后、验证关节变化和发布迟到之前采集：因此服务返回失败、响应状态不合法、客户端拒绝下发及发布迟到退出的最后一次调用不会从计时数据中消失。通信异常保留客户端总耗时，服务端数据留空。
计时文件中的 `call_success` 只表示服务响应 success，不表示该帧已发布或通过了客户端安全检查。

为减少回放中的磁盘操作，计时行暂存在内存，在正常退出或 Python 异常的 finally 中保存、统计；实机模式先尝试停止 tele，再写计时统计。强制杀进程、断电可能丢失尚未保存的计时；现阶段不是崩溃安全日志。

## 记录什么

所有单位为微秒（us），使用单调时钟，不改变每帧求解次数。

| 列 | 口径 |
| --- | --- |
| `ik_latency_us` | Python 的 ROS 服务往返时间，包括通信及服务处理；不再包含终端打印 |
| `selector_call_us` | 服务端围绕一次 selector.select 的计时；不含请求转换、历史准备与接受结果处理 |
| `selector_elapsed_us` | 运动学库内部记录的总时间 |
| `context_build_us` | 库内目标上下文阶段 |
| `phi1_us` | 库内可行臂角域阶段 |
| `phi2_us` | 库内腕部性能域相关阶段 |
| `motion_selection_us` | 库内运动代价与候选选择阶段 |
| `offset_execution_us` | 库内 Offset 执行解相关阶段 |
| `actuator_conversion_us` | 库内最终执行器转换相关阶段 |

模块粒度与现有 Mac 数值回放导出的阶段计时一致，直接透传已有 diagnostics。
不同策略走不同分支，这些字段不是每个函数互斥、无遗漏的 CPU 计时；不能直接求和画成占比总和 100%。
例如腕部择优、偏置细化或时间策略可能包含在现有阶段范围内，本次没有虚构它们的独立耗时。
阶段为零只能表示未记录或未到达该阶段，不能证明该模块执行只用了零时间。

上位机仍使用 `execution` 模式；不为了计时打开额外数值复核。与 Mac 的 `numerical_validation` 模式比较时，应注明两者检查工作量不同。设备安装的库必须包含上述 diagnostics 字段；本次没有远程核实已安装版本。

## 统计口径

每个模块输出 `count / missing_count / zero_count / mean / p50 / p95 / p99 / max`。
P50/P95/P99 使用排序样本的线性插值。阶段列的零值不参与均值和分位数；缺失或不合法服务端计时留空，不伪装成零。

- `all_calls`：全部调用，包含首帧。
- `startup_calls`：第一次调用，单列观察启动影响。
- `moving_non_hold`：非首帧、目标运动、服务成功且未返回 HOLD。
- `moving_hold`：非首帧运动目标的 HOLD。
- `static_calls`：非首帧静止目标。
- `failed_calls`：非首帧服务返回失败或通信异常。
- `unavailable_timing`：非首帧成功调用但服务端计时不可用。

正常运动求解的耗时比较优先使用 `moving_non_hold`，同时报告运动 HOLD 和失败调用数量；不要把静止帧混进去压低均值。
`all_calls` 和 `startup_calls` 与其他组有覆盖关系，不要将所有分组相加。
一轮回放只运行一种方法，文件带有 method。统计是描述性的，不代表线程调度下的硬实时保证。

## 本地验证

```bash
PYTHONPATH=vptele python3 -m unittest discover -s test -p 'test_ik_timing.py' -v
```

本地纯 Python 测试覆盖字段对应、异常/缺失数据、HOLD/静止/首帧分组、分位数、文件防覆盖，以及实际回放客户端类的成功、拒绝、通信异常路径（ROS I/O 替身，不是 ROS 通信测试）。
Mac 未完成 ROS/catkin 编译和设备端通信验证；上机部署必须完成上述编译、重启并先核对 `_timing.csv` 中 `timing_valid=True`。
