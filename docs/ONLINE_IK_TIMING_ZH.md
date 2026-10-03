# 右臂在线回放：逐帧求解耗时记录

更新：2026-10-03，计时 schema 2。适用分支 `for_cjp`，A1 与 `minimum_sufficient_continuity_refined`。

修改前通过 HTTPS `git fetch origin` 确认上位机仓库本地与远端均为
`95f17b080ee398d013e4e85732a4f4c248043fee`，工作区干净。
本次在 Mac 修改上位机日志及运动学库内部计时；未连接设备，未修改下位机。
运动学库已有的其他本地修改保留，不代表设备已经安装这些改动。

## 五阶段计时（新版）

旧六个模块字段保留，但不能用于五阶段占比。新增互斥计时，单位 µs：

| 阶段 | 新字段 | 范围 |
| --- | --- | --- |
| 1 最小运动基准解 | `stage1_baseline_us` | 目标上下文、JointDomain、各分支 MinMotion、基准 Offset IK、SelectBaseline 及基准可执行性检查 |
| 2 调控条件判断 | `stage2_gate_us` | 基准缺口、进入/维持阈值、饱和/冷却等判断及候选域参数准备 |
| 3 运动允许域 | `stage3_motion_domain_us` | Phi_q 与分支、方向锁 |
| 4 腕部改善候选 | `stage4_wrist_candidates_us` | WristDomain、域内 MinMotion、候选 Offset IK/复核与细化、PrepareCandidates |
| 5 时序选择 | `stage5_temporal_selection_us` | 已准备候选的最终排序、TemporalSelect、基准回退、状态与输出关节赋值 |

每帧还记录：

- `stage1_executed` 至 `stage5_executed`：是否进入阶段，不等于是否最终采用候选。
- `selector_other_us`：入口检查、最终执行器转换、通用结果整理等五阶段外开销。
- `timing_schema_version=2`、`paper_timing_valid`：新计时版本与适用标记。
- `selector_mode`、`wrist_skip_reason`：正常/静止/Guard/HOLD，以及饱和、冷却、改善域空、实际收益不足等原因。`none` 不代表必然采用了改善候选。
- `paper_timing_error`：非法数值、执行标记矛盾或加总不闭合时填写；总耗时仍保留，错误阶段值不参与统计。

逐帧检查：`selector_elapsed_us = 五阶段之和 + selector_other_us`。
IK 已包含在阶段 1、4 中，不能再重复相加。`selector_call_us` 是更外层调用计时，
`ik_latency_us` 还包含 ROS 往返；三个总量不能相加。旧阶段字段也不能与新五阶段混加。

### 提前返回与 A1

冻结策略因饱和、冷却等返回基准时，1、2、5 有计时，3、4 未执行且为零。
执行搜索后未采用候选，搜索耗时仍计入。基准失败时只记录已执行阶段。
静止帧在五阶段之前返回，五个执行标记均为 false，实际工作归入 other。

A1 的现有实现仍会认证基准运动域、准备基准候选；这部分累加到阶段 1，
不能把它标为腕部调控触发。A1 阶段 3、4 不执行。这是按功能累计，
不要求阶段 1 在源码中只是一块连续代码。

五阶段适用于 A1 与正常 minimum-sufficient 路径。Guard、continuity-first、
stateful、offset-aware 或额外归因扫描的 `paper_timing_valid=false`，不混入五阶段统计；
其总耗时仍记录，求解行为不变。

### 新统计口径

各新阶段 `mean/p50/p95/p99/max` 使用**实际执行帧**，包括时钟分辨率导致的真零。
`executed_count/skipped_count/execution_rate` 给出执行次数、未执行次数及比例，
分母为组内新计时有效的调用数。缺失或无效数据不算“未执行”。
`mean_all_calls` 为将有效但未执行帧计零后的摊销均值。
不能把阶段 3、4 的条件均值与其他阶段均值直接相加当作整帧均值。
JSON 的 `paper_time_fractions` 使用同一有效帧集合的累计时间分配，包含 other，合计为 1。
优先比较运动帧；首帧、静止帧和异常 HOLD 单独展示。

### 部署必须配套更新运动学库

仅改本仓库无法补齐计时：计时边界在运动学库 `select()` 内。
必须先更新包含 `SelectorDiagnostics::paper_stage_microseconds` 等字段的运动学库
（本地开发分支 `feature/redundancy-selector`），重新编译安装，再编译本仓库。
配套运动学提交为 `3905f14`（五阶段计时）；本次发布未包含另行保留的本地浮点容差及 no-HOLD 诊断修改。
头文件结构有变化，不能混用新头文件与旧二进制，也不能只换动态库保留旧客户端。
在上位机正确的运动学仓库执行，`build` 替换为该机器既有的构建目录：

```bash
cmake --build build -j4
sudo cmake --install build
```

沿用原 CMake 配置、安装前缀和权限；不是切换下位机 interaction 库。
再按下一节编译上位机、重启所有 ArmIK 服务端及客户端。正常回放命令不变。
本次未更改 CSV、初值或算法参数。默认仍为
`experiments/r80_pose_300413/R80_pose_300413_engineering.csv`，SHA256：
`40c71bce5f56c634bbc52c1e553b60dcd919a865e980d5cc288277d5c0e79e96`。
若使用环境变量或 `--input` 覆盖路径，以 audit 为准。

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

## 总耗时与兼容保留的旧模块字段

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

上述旧模块字段直接透传历史 diagnostics，仅保留兼容用途。
不同策略走不同分支，这些旧字段不是互斥、无遗漏的阶段计时，不能相加画占比。
旧字段为零仍可能表示漏记或未到达，不用于推断触发频率。
论文五阶段改用本页开头列出的新字段和执行标记。

上位机仍使用 `execution` 模式；不为了计时打开额外数值复核。与 Mac 的 `numerical_validation` 模式比较时，应注明两者检查工作量不同。设备安装的库必须包含上述 diagnostics 字段；本次没有远程核实已安装版本。

## 统计口径

每个模块输出 `count / missing_count / zero_count / mean / p50 / p95 / p99 / max`。
P50/P95/P99 使用排序样本的线性插值。只有旧阶段列排除零；新五阶段根据执行标记统计，
真零仍保留。缺失或不合法计时留空，不伪装成零。

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
2026-10-03 本地验证：Python 计时测试 10/10 通过，包含新字段映射、提前失败保留记录、
真实零耗时与未执行区分、条件/摊销均值、无效计时不污染总时间等。
运动学库及回放目标编译通过，选择器测试通过（包含新提前返回/加总断言）。
R80 pose 300413 同一输入、两种方法各 1186 帧，计时修改前后全部非计时字段一致；
五阶段加 other 逐帧闭合。数值回放的首帧为初始化记录，不当作真实 selector 调用。

Mac 未完成 ROS/catkin 编译和设备端通信验证。部署后核对 `timing_schema_version=2`；
正常运动帧 `timing_valid=True`、`paper_timing_valid=True`、`paper_timing_error` 为空，
并核对实际 R80 CSV 哈希。Guard 等不适用路径应为 `paper_timing_valid=False`。
