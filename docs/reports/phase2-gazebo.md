# 阶段二：Gazebo Harmonic Panda 物理抓放报告

## 被测提交与环境

- 分支：`sim/gazebo-panda-pick-place`
- 本报告所测代码提交：`5cda5f2cae9288d3125e96181678d3b37364381b`
- 阶段二首个功能提交：`d89fb2719f9ca91c5de2a14c8aa6d5d13b02c4bf`
- 上游基线：`40353703cc13eb1c53434adc62cd749e283af1e6`
- 固定种子批测开始前，`git status --porcelain` 为空；完整回归前删除了本地 `build/`、`install/`、`log/` 并从头构建。仿真逐次日志与 JSON 在测试后才复制进仓库。
- 测试时间：2026-10-04。20 次批测汇总时间为 `2026-10-04T07:18:57.960985Z`。
- 宿主机：Ubuntu 22.04.5 LTS、Linux `6.8.0-138-generic`、Intel Core i5-14600KF（20 逻辑 CPU）、31 GiB RAM、Docker Engine 29.3.1。宿主机已有 ROS Noetic/Humble；本次没有修改宿主 ROS。没有可用 NVIDIA 驱动；未使用 GUI/GPU，Gazebo 以 headless CPU 方式运行。
- 容器：Ubuntu 24.04.5 LTS、ROS 2 Jazzy、Python 3.12.3、GCC 13.3.0、CMake 3.28.3、colcon-core 0.21.3。镜像 `ros:jazzy-ros-base` 固定 OCI digest `sha256:066420e07f60aa18262f2479981def87ebcfcec42eefb0c0c57c4a46098348ca`。

| 依赖 | 实际版本 |
|---|---|
| MoveIt 2 (`moveit_core`) | 2.12.4 |
| BehaviorTree.CPP | 4.10.0 |
| ros2_control (`controller_manager`) | 4.48.0 |
| Gazebo Sim / Harmonic | 8.15.0 |
| `gz_ros2_control` | 1.2.20 |
| `ros_gz_sim` | 1.0.24 |

APT 软件源为 USTC Ubuntu/ROS 镜像，Dockerfile、环境检查脚本及 288 个 ROS deb 的实际版本清单分别见 `docker/jazzy.Dockerfile`、`scripts/check_jazzy_environment.sh` 和 [`ros-jazzy-packages.txt`](ros-jazzy-packages.txt)。镜像 digest 固定；apt 包由构建时软件源解析，软件源不是不可变快照。

## 验收定义

配置位于 `src/robot_panda_gz_sim/config/runtime.yaml`：任务超时 30 s、停止确认 5 s、放置 XY 容差 0.05 m、Z 容差 0.015 m、放置后稳定观测窗口 500 ms。观测窗口要求仿真时间戳推进；暂停时重复读缓存不算新证据。时间回退或恢复会切换样本 epoch 并清除旧接触/位姿缓存。

抓取成功要求双指物理接触并有独立 Gazebo 物体位姿抬升证据；释放后要求新鲜物体位姿在窗口中稳定。重复批测还独立核验左右指接触、最终已释放、Gazebo `tray_floor` 接触以及终态 XY/Z 误差。仅控制器 Action 成功或 MoveIt 规划场景附着不构成成功。物体由 DART 接触动力学移动，没有瞬移逻辑。位姿来源是明确标记的 Gazebo 仿真真值，不是真实视觉。

## 完整构建和回归

在被测提交 `5cda5f2...`、跟踪工作区干净的状态执行：

```bash
rm -rf build install log
scripts/with_jazzy.sh bash scripts/test_all_jazzy.sh
```

总命令退出码 **0**。完整输出见 [`phase2-regression-final.log`](phase2-regression-final.log)（372 KiB）。`scripts/check_jazzy_environment.sh` 退出码 0，输出见 [`phase2-environment-check.log`](phase2-environment-check.log)。

| 实际步骤 | 结果 | 退出码 |
|---|---:|---:|
| `bash scripts/test_core.sh` | 62 项通过 | 0 |
| `colcon build --event-handlers console_direct+ --cmake-args -DBUILD_TESTING=ON` | 7 个包构建完成，33.1 s | 0 |
| `colcon test --event-handlers console_direct+ --return-code-on-test-failure` | 全部测试目标完成 | 0 |
| `colcon test-result --verbose` | 4 个测试条目，0 错误、0 失败、0 跳过 | 0 |
| `python3 scripts/test_ros.py` | 37 项通过，40.563 s | 0 |
| `python3 scripts/test_panda.py` | 11 项通过，64.376 s | 0 |

最初在 `d89fb27` 上运行完整回归时，ROS 集成测试 37 项中有 5 项因 `perception_frame` 被重复声明而失败（总命令退出码 1）。问题是 Gazebo 后端和原生位姿后端共用参数声明；提交 `5cda5f2` 将这项声明限定到需要它的 Gazebo 后端，同时保留原生位姿参数契约。没有删除断言或放宽检查。初次失败的原始记录保存在 [`phase2-regression-initial-d89fb27.log`](phase2-regression-initial-d89fb27.log)；修复后完整干净构建及测试全部通过。

## 固定种子重复仿真

执行命令：

```bash
scripts/test_gazebo_trials.py --count 20 --seed-start 100
```

该脚本为每次运行实际调用 `scripts/with_jazzy.sh scripts/test_gazebo.sh --timeout-ms 30000 --seed <seed> ...`，不重试失败。批测汇总记录代码提交 `5cda5f2cae9288d3125e96181678d3b37364381b`，开始时工作区状态为空。20 次均产生结果 JSON 和完整 Gazebo 日志，原始证据位于 [`phase2-trials/`](phase2-trials/)；机器可读汇总为 [`phase2-trials-summary.json`](phase2-trials-summary.json)。

| 指标 | 实测 |
|---|---:|
| 完整运行 | 20 / 20，seed 100–119 |
| 独立验收通过 | 19 / 20（95%） |
| 任务报告成功但独立审计不通过 | 0 |
| 成功任务 XY 放置误差 | 平均 3.121 mm，最大 7.742 mm（19 次） |
| 成功任务 Z 放置误差 | 1.000 mm（19 次；每次满足 15 mm 门限） |
| 20 次全部可测 XY 误差 | 0.542–11.068 mm；包含 seed 101 失败终态 |

seed 101 以退出码 1 结束，状态为 `failed/MOTION_FAILED`，因此整个批测脚本也按设计返回 **1**（未达到要求的 20/20 全通过）。它通过过双指接触，但放置动作阶段手臂轨迹未到达停止状态；Gazebo 日志记录 Panda joint 1 位置误差 20.070 mm、joint 5 误差 11.888 mm，均超过 ros2_control 配置的 1 mm 容差。物体仍被夹持，终点高于托盘底 17.173 mm，没有托盘底接触，故不满足释放、托盘接触和 Z 误差条件。可用 seed 101 和汇总中给出的命令复现；该运行记录在 `phase2-trials/trial-101.json` 与 `.log`。这是场景/轨迹跟踪可靠性问题，具体是规划路径、DART 动态或控制器容差/跟踪之间的哪一项尚未分离定位。

随后在代码 SHA 不变且工作区干净时，以同一 seed 101 单独诊断重放：

```bash
scripts/with_jazzy.sh scripts/test_gazebo.sh --seed 101 --timeout-ms 30000 \
  --result-json docs/reports/phase2-replay-seed-101.json \
  --launch-log docs/reports/phase2-replay-seed-101.log
```

这次任务成功，退出码 0，XY 误差 5.831 mm、Z 误差 1.000 mm，且有双指接触、释放和托盘底接触证据。它是额外诊断结果，不计入 20 次批测的 19/20 成绩。两次相同 Gazebo seed 得到不同终态，说明 `--seed` 固定的是 Gazebo 世界种子，尚未固定 MoveIt/OMPL 的规划随机数流；本仓库没有给 OMPL 配置随机种子。OMPL 的随机数种子需要在实例生成前设置，默认种子取决于启动时钟（见 [OMPL RNG API](https://ompl.kavrakilab.org/RandomNumbers_8cpp_source.html)）。因此当前批测是固定 Gazebo seed 的重复测试，不保证规划轨迹逐次相同；失败可以由 seed 101 单独触发，但不能声称它在相同 seed 下稳定复现。

## 指定故障场景与时钟边界

下表来自独立注入场景，结果 JSON 与启动日志均保留在 `docs/reports/`。故障注入场景以非零退出码表示任务按预期失败，并非测试框架跳过。

| 场景 | 实际结果 | 证据 |
|---|---|---|
| 默认物体扰动 `(+15, -10) mm`，seed 43 | 成功；终态 XY 误差 0.752 mm | `phase2-gazebo-perturb-43.{json,log}` |
| Gazebo 暂停 1 s 后恢复，seed 54 | 两个 pause/unpause 服务均返回 `data: true`；恢复后任务成功。另有 C++ 时间源测试确认暂停时重复 stamp 不会生成新 sample ID | `phase2-gazebo-pause-54.{json,log}` |
| 注入抓空，seed 52 | `failed/GRIPPER_VERIFICATION_FAILED`；方块仍在桌面，无双指抓取证据 | `phase2-gazebo-missed-grasp-52.{json,log}` |
| 错误放置偏置 `+80 mm`，seed 48 | `failed/MOTION_FAILED`；终态距托盘中心 XY 81.176 mm，仅有桌面接触，无托盘底接触 | `phase2-gazebo-wrong-place-48.{json,log}` |
| 取消，seed 45 | 取消被接受，停止得到测量确认；请求至结果 0.885 s（小于 5 s） | `phase2-gazebo-cancel-45.{json,log}` |
| 任务超时 3 s，seed 46 | `timed_out/TIMEOUT`；停止得到测量确认，任务墙钟 3.599 s | `phase2-gazebo-timeout-46.{json,log}` |
| 丢弃抓取反馈，seed 47 | `faulted/STOP_UNCONFIRMED`；停止期限耗尽，资源继续保留，不伪报停止 | `phase2-gazebo-feedback-loss-47.{json,log}` |
| 执行中重置世界，seed 53 | Gazebo reset 服务成功，但控制栈重启时无法确认停止；`faulted/STOP_UNCONFIRMED` 并保留资源 | `phase2-gazebo-reset-53.{json,log}` |

C++ 和 Python 时间源/传感器新鲜度契约测试均包含在上述 colcon 测试中，覆盖重复、过期、未来时间戳及仿真时间回退。暂停/恢复实际测试验证没有依靠重复缓存样本补足稳定窗口。世界重置场景的控制器管理器会重启，现状是 fail-closed；任务不会自动恢复，重置中的在途动作不能确认停止时会锁住资源。这是已知限制，不计为通过的重置恢复。

## 结果、限制与复核

- `robot_core` 仍不依赖 ROS、Gazebo 或模型 SDK。技能拥有抓取/放置语义、阶段和结果校验；ROS/MoveIt、时间戳、Gazebo 真值与接触传感器通过适配层接入。原有 GenericSystem/mock 路径仍独立保留。
- Gazebo DART 启动日志提示不支持 URDF mimic constraint。测试能观测双指接触，但该警告和 19/20 的批测结果要求继续检查夹爪模型、接触参数和轨迹跟踪。
- seed 101 的失败及同 seed 成功重放均为现阶段应保留的物理闭环证据。该差异显示规划随机性尚未固定，且跟踪失败原因未隔离。不得据此宣称 100% 稳定抓放；批测 95% 也不足以作为实机、可靠性或安全认证证据。
- 反馈中断、运动中重置下 `STOP_UNCONFIRMED` 会保留资源，是保守故障策略。操作者需重启/复位控制栈并重新检查世界状态；当前没有自动恢复机制。
- 日志里还存在 `ROS_LOCALHOST_ONLY` 弃用警告、MoveIt 无 3D sensor plugin 的提示，以及 ros2_control 关闭阶段统计线程警告。场景明确使用 Gazebo 真值，不提供真实 3D 感知；这些警告没有令测试退出失败，但仍应由审查者判断是否要消除。
- 证据校验值清单为 [`phase2-artifacts.sha256`](phase2-artifacts.sha256)。批测原始 JSON/日志约 1.8 MiB，阶段二回归及指定场景日志也一并提交；没有录像、rosbag、模型权重、容器镜像、构建缓存或凭据。

独立审查可优先检查 `src/robot_panda_gz_sim/worlds/panda_pick_place.sdf` 与控制器配置、`gz_scene_sync.py` 的真值/接触和 MoveIt 附着、`robot_core` 抓放后置验证及资源停止确认，以及 seed 101、反馈中断和重置场景的原始日志。
