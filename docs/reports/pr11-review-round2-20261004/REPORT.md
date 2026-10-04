# PR #11 第二轮审查修复、回归与物理复测

## 提交与范围

- PR：[molong-d/robot_codex#11](https://github.com/molong-d/robot_codex/pull/11)，基分支 `framework/target-resolution-v04`，开发分支 `sim/gazebo-panda-pick-place`，保持 Draft。
- 本轮被测代码 SHA：`f92d5325d13505f14a3d24767fdc4dab9b314560`。父代码提交为 `77a48057a528c5579fa3caf6d15ac2b2ee0124d9`；随后仅新增故障注入时序回归。测试时跟踪工作区干净，20 次批测及 9 个定向场景均指向该代码 SHA。
- `143a5b2230310014d2a3a7b58585b0a818666e17` 及更早报告、`backup/phase2-gazebo-20261004`、此前 19/20 和 18/20 结果均保留，没有覆盖或重释。本报告记录的是新运行，不把历史数据当作本轮物理结果。
- 本轮只处理接触语义、运行时场景同步门控、释放后稳定窗口及离线证据可复算性；未加入真实视觉、模型 SDK、力控或真机功能。

## 环境

实际测试在 Ubuntu 22.04.5 宿主机上的隔离 Ubuntu 24.04.5 / ROS 2 Jazzy 容器中完成。宿主机 Docker Engine 为 29.3.1，CPU 为 Intel Core i5-14600KF（20 逻辑 CPU），内存 31 GiB；NVIDIA 驱动不可用，Gazebo 使用 headless CPU 模式。容器镜像 ID 为 `sha256:e78120951b67089bbad339217778e865734801e5faa4d360c13e70e1dd40d7d1`，Dockerfile 基础镜像固定到 `ros:jazzy-ros-base@sha256:066420e07f60aa18262f2479981def87ebcfcec42eefb0c0c57c4a46098348ca`。apt 包从构建时 Jazzy/Ubuntu 软件源安装，软件源不是不可变快照。完整版本见 [`evidence/ENVIRONMENT.txt`](evidence/ENVIRONMENT.txt)。

## 三项修复与回归

1. **接触状态分离**：核心新增 `unknown / none / left_only / right_only / both`。夹稳只接受新鲜、身份/epoch 匹配的双指接触及抬升；释放只接受动作后的新鲜双指无接触。单指接触和未知反馈均不能用于释放，也不会清除现有规划场景附着。接口版本及迁移说明更新于 [`GraspContact.msg`](../../../src/robot_interfaces/msg/GraspContact.msg) 与 [`versions.md`](../../versions.md)。运行时适配在 `robot_core`、ROS adapter、`ParallelGripper`、`GazeboWorldAdapter` 和 `gz_scene_sync.py` 使用相同语义。
   - 回归：接触语义单测 5/5，覆盖左右单指、双指、none、过期、乱序、跨 epoch 和单指仍在目标区；释放后接触反馈中断场景 seed 703 在验证阶段返回 `INVALID_VERIFICATION_EVIDENCE`，而非把缺失消息解释成释放。
2. **场景同步门控进入执行路径**：ROS-free `PlanningSceneGate` 契约由技能层在每段依赖碰撞场景的运动下发前检查；Gazebo ROS adapter 提供 epoch、版本、期望/确认附着、请求中状态和 producer 身份。`/execute_task` 与 `/execute_plan` 走同一门控路径。取消/超时等待不阻塞单线程执行器；确认不匹配、服务失败、reset、迟到响应和未确认停止均 fail closed。核心接口及技能实现见 `src/robot_core/include/robot_core/components.hpp`、`skills.hpp`，场景协调见 `src/robot_panda_gz_sim/scripts/gz_scene_sync.py`。
   - 回归：直接任务入口未就绪、精确版本确认后继续、取消/超时、reset 迟到响应，以及 attach 延迟阻止抬升、detach 延迟阻止撤离均纳入核心/场景状态测试。正式物理运行中发现的 finger-tray 起始碰撞属于规划碰撞检查拒绝，未被 ready 信号绕过。
3. **稳定窗口从释放事件起算**：独立审计器版本 `2.0.0` 只从匹配 run/object/epoch 的释放事件之后累加连续样本；窗口内逐样本核验新鲜位姿、目标、接触、配对时间、tray 支撑、无手指接触、位置及仿真源时间速度。离开区域、接触异常、时间回退或间隔过大都会清空/拒绝窗口。规则见 [`scripts/gazebo_evidence.py`](../../../scripts/gazebo_evidence.py)，唯一验收配置见 [`acceptance.json`](../../../src/robot_panda_gz_sim/config/acceptance.json)。
   - 回归：审查复现“释放前 500 ms 稳定、释放后仅 10 ms”在修复前曾错误通过；新测试拒绝该数据。释放后不足样本/时长拒绝，完整连续窗口通过；单指接触、离开目标区、反馈缺失、时间回退、历史 tray 接触和错误身份均有测试。审查前失败输出保存在 `evidence/red-reproductions/`。

## 完整软件回归

在代码 SHA `f92d5325d13505f14a3d24767fdc4dab9b314560`、干净跟踪工作区中执行：

```bash
bash scripts/with_jazzy.sh bash scripts/test_all_jazzy.sh
```

退出码 **0**。汇总如下，原始命令输出在 [`full-regression.log`](evidence/logs/full-regression.log)：

| 项目 | 实际结果 |
|---|---|
| `scripts/test_core.sh` | 66/66 通过 |
| 完整 ROS 工作空间 `colcon build`（`BUILD_TESTING=ON`） | 7 个 package 构建完成 |
| `colcon test` / `colcon test-result --verbose` | 10 项，0 errors、0 failures、0 skipped |
| 新增 Gazebo CTest 专项 | sensor timestamp、scene state、trial identity、contact semantics 5 项、release window audit 9 项、offline package roundtrip 1 项、fault order 2 项均通过 |
| `scripts/test_ros.py` | 37/37 通过 |
| `scripts/test_panda.py` | 11/11 通过 |

退出日志中保留了负向测试预期的错误输出，以及 ROS shutdown 时 `controller_manager.pal_statistics` 的上下文关闭警告；测试进程退出码为 0。没有将测试改为跳过或放宽物理验收容差。

## 定向物理场景

命令（编排器在宿主机运行，每个 trial 由它单独启动 Jazzy 容器）：

```bash
python3 scripts/test_gazebo_scenarios.py --output-dir .review-runs
```

批次 `scenarios-20261004T150945Z-5f83c11c05`，代码 SHA 匹配，场景编排退出码 **0**，9/9 场景检查符合预设；详细 run_id、退出码、状态、执行步骤、资源、计时与断言见 [`SCENARIO_OUTCOMES.json`](evidence/SCENARIO_OUTCOMES.json) 和 [`PHYSICAL_RUN_INDEX.json`](evidence/PHYSICAL_RUN_INDEX.json)。故障 trial 本身退出码为 1 是预期任务拒绝，场景只有在阶段、错误码、物理审计及资源行为匹配预期时才计场景通过。

- seed 700 正常抓取放置：任务成功、独立审计通过、XY 误差 0.004317 m、停止已确认且资源清空。
- seed 708 初始位姿扰动：任务和物理审计通过。
- seed 701 错误放置：`place_object` 已成功；释放后才注入偏移，随后 `verify_placement` 返回 `PLACEMENT_NOT_VERIFIED`；物理审计拒绝，资源清空。这是“动作完成但放置条件不满足”的有效验证证据。
- seed 702 抓取后掉落：真实打开模拟夹爪触发掉落，任务以 `GRASP_NOT_VERIFIED` 拒绝并释放资源；未把后续运动或结果验证能力冒充为已覆盖。
- seed 703 释放后中断接触反馈：运行时在放置验证返回 `INVALID_VERIFICATION_EVIDENCE`，停止确认、资源清空；物理记录证明反馈中断确实发生在 release 后。
- seed 704 取消：`cancel_to_result_s=0.277805`，请求至停止确认 `0.265719 s`；结果时刻和停止时刻分别记录，均小于 5 s。
- seed 705 超时：`TIMEOUT` 后 `0.050701 s` 确认停止，资源清空。
- seed 706 暂停/恢复：两个服务成功，恢复后任务与物理审计通过。
- seed 707 时间 reset：reset 服务成功，但停止反馈未能确认；返回 `faulted/STOP_UNCONFIRMED`，锁存故障并保留 arm/gripper 资源，禁止后续动作。场景断言通过代表安全拒绝符合预期，不代表 reset 后恢复运行成功。

## 新正式批测与失败分类

在干净跟踪工作区，以 `f92d5325d13505f14a3d24767fdc4dab9b314560` 执行 20 次正式运行，无自动重试：

```bash
python3 scripts/test_gazebo_trials.py --count 20 --seed-start 200 \
  --process-timeout-s 240 --output-dir .review-runs \
  --batch-id pr11-round2-f92d532-seeds-200-219
```

该批次退出码 **1**，因为 6 个任务 trial 未满足成功门槛；这不是环境启动失败。20/20 都有新的匹配 run_id、seed、代码 SHA 和完整 JSON；0 个 240 s runner 墙钟超时、0 个缺失/损坏/身份错误结果、0 个假成功。14/20 同时满足进程退出码 0、任务成功、独立物理审计通过及安全资源释放，成功率 70%。

| seed | task 结果 | 独立物理审计 | 分类 |
|---:|---|---|---|
| 202 | `TIMEOUT`；`locate_object` 成功，`pick_object` 被取消；停止确认且资源清空 | 拒绝：物体未抬升，XY 距离 0.278196 m | 任务阶段超时 |
| 203 | `GRIPPER_VERIFICATION_FAILED`；`place_object` 开始打开夹爪时未达到新鲜静止释放状态（末次宽度 0.049374 m） | 拒绝 | 夹爪物理/反馈未满足释放要求；DART mimic 警告仍在，当前证据不证明两者存在因果关系 |
| 205 | `MOTION_FAILED`，释放后撤离规划失败；停止确认且资源清空 | 拒绝：稳定窗口观测 476 ms，小于配置的 500 ms | 规划场景将 `panda_rightfinger` 与 `sim_tray` 判为接触，MoveIt 起始碰撞检查中止 |
| 207 | `MOTION_FAILED`，释放后撤离规划失败；停止确认且资源清空 | 物理终态独立审计通过，但任务动作失败故不计成功 | MoveIt `CheckStartStateCollision` 报 `panda_leftfinger - sim_tray` |
| 209 | `MOTION_FAILED`，释放后撤离规划失败；停止确认且资源清空 | 物理终态独立审计通过，但任务动作失败故不计成功 | 同为 `panda_leftfinger - sim_tray` 起始碰撞 |
| 217 | `MOTION_FAILED`，释放后撤离规划失败；停止确认且资源清空 | 物理终态独立审计通过，但任务动作失败故不计成功 | `panda_rightfinger - sim_tray` 起始碰撞 |

因此 6 个非零子进程退出中，3 个同时被独立物理审计拒绝，另外 3 个虽测得物体终态物理放置合格，完整任务仍因撤离动作失败而拒绝。三类都不是正常任务成功。批次指标：审计失败 3、无效结果 0、误报成功 0。14 个验收成功运行的 XY 误差中位数 0.002987 m、最大 0.009088 m；20 次末态位姿均可测量，但失败运行的末态偏差不作为成功精度统计。

## seed 700 / 202 / 207 的边界诊断

- **700**：定向正常场景唯一正式记录 `trial-001-seed-700-0a6a7238739a` 成功、退出 0，物理审计通过，XY 误差 0.004317 m。
- **202**：正式批测 `trial-003-seed-202-25c5122c4281` 在 `pick_object` 达到任务 30 s 超时并确认停止，进程未触发 240 s runner timeout。独立诊断重跑 `trial-001-seed-202-c68e4fe322f4` 通过（退出 0、XY 0.007455 m）；保留正式失败，不用重跑替代。Gazebo seed 不固定 OMPL 规划随机流，不能承诺同 seed 同轨迹。
- **207**：正式批测 `trial-008-seed-207-861e8229ef47` 在 release 后撤离规划阶段失败；Gazebo 日志明确为 `CheckStartStateCollision` 报 `panda_leftfinger - sim_tray`，不是本次证据所见的夹爪等待或轨迹跟踪误差。独立诊断重跑 `trial-002-seed-207-99338621e577` 通过（退出 0、XY 0.003382 m），仍保留正式失败。问题现阶段表现为接触/碰撞场景边缘条件对轨迹敏感，未证明其已消除。

所有碰撞失败的正式日志在本机 `.review-runs/pr11-round2-f92d532-seeds-200-219/`；审查必要的 seed 207 碰撞日志行也摘录到 [`SEED_DIAGNOSTICS.json`](evidence/SEED_DIAGNOSTICS.json)。DART mimic constraint 警告未隐藏。

## 可离线复算证据包

Git 内提交 [`pr11-round2-f92d532-audit-evidence.tar`](evidence/pr11-round2-f92d532-audit-evidence.tar)，无需 ROS/Gazebo 即可重算。它包含 31 个同代码 SHA 的运行：20 个正式 trial、9 个定向物理场景和 2 个单独诊断重跑；完整 run_id/seed/子进程退出码及任务状态在清单中，组别索引见 `PHYSICAL_RUN_INDEX.json`。JSONL gzip 保留每个运行的位姿、接触、任务阶段样本（包括被拒绝的样本），不只保留成功区间。

| 项目 | 实际值 |
|---|---|
| 被测代码 | `f92d5325d13505f14a3d24767fdc4dab9b314560` |
| 审计程序 | `2.0.0` |
| 导出格式 | `robot-codex-gazebo-audit-jsonl-gzip-v1` |
| 验收配置 SHA-256 | `552ee08315fc77f0ec9474e305a917e80895e50e26fd4e2c32ca8c57b54fad08` |
| 场景预期配置 SHA-256 | `10f43deec7323cdb21b506ab392e2f4e73419c92b8212e86d7d0ef318bb719d9` |
| 运行数 / JSONL 行数 | 31 / 718,209 |
| 未压缩样本大小 / gzip 样本大小 | 346,178,134 / 11,821,113 bytes |
| tar 大小 | 11,898,880 bytes |
| tar SHA-256 | `4ceb2a84dde56f55cc1540ae86d0b3783c28f11e3d197530052091fc891302aa` |
| 原始结果与导出样本审计比较 | 31/31 一致；`all_recomputations_match=true` |

导出器在生成包时先从原始 JSON 样本重算，再从 gzip 包独立离线重算并比较；另在打包后再次执行以下命令，退出 0：

```bash
python3 scripts/recompute_gazebo_evidence.py \
  docs/reports/pr11-review-round2-20261004/evidence/pr11-round2-f92d532-audit-evidence.tar
(cd docs/reports/pr11-review-round2-20261004/evidence && \
  sha256sum -c pr11-round2-f92d532-audit-evidence.tar.sha256)
```

重算 JSON、物理运行索引、场景结果、seed 诊断、测试日志及哈希清单均在 `evidence/`。原始高频 Gazebo 控制台日志仍留在运行主机被忽略的 `.review-runs/`，没有假称它们可从本报告下载；独立物理审计必需的样本已包含在提交包中。

## 复核限制与运行命令

- 目前 Gazebo DART 仍报告不支持 Panda URDF mimic constraint；正式批测中有夹爪释放确认失败以及手指/托盘起始碰撞导致的撤离规划失败。审计拒绝 6/20 任务，不能宣称 20/20 可靠。
- Gazebo 的对象位姿为明确标记的仿真真值；无真实视觉、机器人 SDK 或完整力控。没有真机验证或安全认证。
- 组件/场景门控、接触新鲜度与离线审计单测通过不替代跨分布物理稳定性评估。seed 200–219 是新的正式批测；700/202/207 诊断单独计数，seed 202/207 的重跑不替换正式失败。
- 从干净 clone 复核软件回归：

```bash
git clone https://github.com/molong-d/robot_codex.git
cd robot_codex
git checkout sim/gazebo-panda-pick-place
bash scripts/with_jazzy.sh bash scripts/test_all_jazzy.sh
```

- 要另跑新的 9 场景和 20 次正式批测，请在宿主机执行编排器（每次 trial 会启动 Jazzy 容器）：

```bash
python3 scripts/test_gazebo_scenarios.py --output-dir .review-runs
python3 scripts/test_gazebo_trials.py --count 20 --seed-start 200 --process-timeout-s 240 --output-dir .review-runs
```

上述复测会产生新的 run_id 和 Gazebo 随机过程，不会复现本报告结果；对已提交证据包的离线复算无需 Docker、ROS 或 Gazebo。
