# PR #11 审查问题修复与物理仿真复测

## 提交和工作区

- PR：[#11](https://github.com/molong-d/robot_codex/pull/11)，基分支 `framework/target-resolution-v04`，本次继续更新 `sim/gazebo-panda-pick-place`，保持 Draft。
- 被测代码提交：`8c3649548b6b6b944838dd9c62aa01be9d1b4dd8`。本报告及证据由后续独立报告提交加入；报告提交不属于被测代码 SHA。
- 被测前后的跟踪工作区均干净。代码提交后未修改源代码；本次报告提交只增加文档和证据。
- 远程开发分支初始头为已知报告提交 `b9da6dbfa2a81739a57e2fb9cec7b116a6a085a2`，未发现远程后续修复。旧备份 `backup/phase2-gazebo-20261004` 仍指向 `5cda5f2cae9288d3125e96181678d3b37364381b`，历史证据未覆盖。

## 环境

| 项目 | 实际值 |
|---|---|
| 宿主机 | Ubuntu 22.04.5 LTS，Linux 6.8.0-138-generic |
| CPU / 内存 | Intel Core i5-14600KF，20 逻辑 CPU；31 GiB RAM |
| 显示 / GPU | 无可用 NVIDIA 驱动；无显示环境；本轮 headless CPU 仿真 |
| 隔离环境 | Docker 内 Ubuntu 24.04.5、ROS 2 Jazzy；没有改动宿主机已有 ROS 工作区 |
| Python / GCC / CMake | 3.12.3 / 13.3.0 / 3.28.3 |
| MoveIt 2 / BehaviorTree.CPP | 2.12.4 / 4.10.0 |
| ros2_control / Gazebo Sim | 4.48.0 / Harmonic 8.15.0 |
| gz_ros2_control / ros_gz_sim | 1.2.20 / 1.0.24 |
| ROS 镜像 | `ros:jazzy-ros-base` OCI digest `sha256:066420e07f60aa18262f2479981def87ebcfcec42eefb0c0c57c4a46098348ca`；项目测试镜像由仓库 `docker/jazzy.Dockerfile` 构建 |

软件版本由仓库环境脚本和 apt 包清单记录。镜像基础 digest 固定；构建时 Ubuntu/ROS apt 镜像是可变源，不应理解成不可变 apt 快照。Gazebo 日志中的 DART mimic constraint 不支持警告仍保留。

## PR 五项审查问题

1. **批测新鲜度、身份和退出码**：`scripts/test_gazebo_trials.py` 为 batch 和 trial 创建不可复用目录及唯一 `run_id`，每条结果必须匹配 `run_id`、seed、代码 SHA，且完整、任务成功、进程退出码为 0、独立物理审计通过，才计入成功。子进程有墙钟超时，诊断写入独立日志。新增回归在旧目录预放成功结果、再让新启动返回 127 且不生成 JSON；旧成功不得被复用，批测必须失败。见 `src/robot_panda_gz_sim/test/test_gazebo_trials.py` 和 `evidence/logs/trial-driver-unit.log`。
2. **未知接触不再当作无接触**：`GazeboWorldAdapter::placement` 区分接触、无接触和未知；放置验收要求来源、epoch、动作后时间和新鲜度都匹配。位姿、目标和接触各自维护样本身份与年龄。seed 703 在真实释放后才停止接触反馈，运行时在 `verify_placement` 以 `INVALID_VERIFICATION_EVIDENCE` 拒绝；不是从仿真启动时就关闭反馈。
3. **消息乱序和时钟回退分开处理**：C++/Python 时间源拒绝正常前进时钟下的旧 source stamp，不刷新 sample id；由确认的仿真时钟回退推进 epoch，并阻止旧 epoch 回调/缓存参与组合证据。重复 stamp、乱序、过期、未来、暂停、恢复、真实回退和重置后迟到旧消息有定向单测。仿真时间用于物理速度/稳定窗口；单调墙钟用于通信接收新鲜度、进程/停止超时和取消时长。暂停时相同缓存 sample 不增加证据。
4. **MoveIt 场景确认与期望状态分开**：场景协调器记录提交请求的状态快照、版本和 epoch，并分别维护 desired 与 confirmed 状态。等待期间目标变化后会继续协调；失败/超时不会置 ready；reset 会发送显式附着清理，迟到旧响应不能令当前场景 ready。保留现有 ACM 条目并仅合并必要碰撞对。定向测试覆盖 attach/detach 交错、reset/迟到响应、服务失败、超时和未就绪。
5. **独立物理审计不依赖运行时成功位**：运行结果保存含 frame、source stamp、接收单调时间、epoch、序列号的物体/目标/接触样本和阶段事件。审计要求同一有效时刻左右指接触、随后抬升、释放后的新鲜无指接触、最终 tray-floor 接触、位置容差及新鲜稳定窗口；拒绝旧/错坐标系/跨 epoch 样本和历史接触冒充最终接触。测试还验证不读运行时 success 布尔值。单一验收配置是 `src/robot_panda_gz_sim/config/acceptance.json`，场景预期在 `scenario_expectations.json`。

## 回归结果

实际运行命令（由 `scripts/with_jazzy.sh` 在上述隔离 Jazzy 环境执行）：

```bash
scripts/with_jazzy.sh bash scripts/test_all_jazzy.sh
```

脚本内部的命令及退出码：

| 命令 | 结果 | 退出码 |
|---|---|---:|
| `bash scripts/test_core.sh` | 62 项通过 | 0 |
| `colcon build --event-handlers console_direct+ --cmake-args -DBUILD_TESTING=ON` | 完整工作空间构建成功 | 0 |
| `colcon test --event-handlers console_direct+ --return-code-on-test-failure` | 所有测试目标执行完成 | 0 |
| `colcon test-result --verbose` | 6 tests，0 errors / failures / skips | 0 |
| `python3 scripts/test_ros.py` | 37 项通过 | 0 |
| `python3 scripts/test_panda.py` | 11 项通过，60.194 s | 0 |

此外在该 SHA 上直接运行新增专项测试：

```bash
python3 src/robot_panda_gz_sim/test/test_sensor_timestamps.py
python3 src/robot_panda_gz_sim/test/test_scene_state.py
python3 src/robot_panda_gz_sim/test/test_gazebo_trials.py
```

三项均退出 0。回归命令输出、colcon 结果、ROS/Panda 测试日志和专项测试日志在 `evidence/logs/`。

## 物理场景复测

场景期望在启动前写入 `src/robot_panda_gz_sim/config/scenario_expectations.json`；物理容差来自 `config/acceptance.json`：XY 0.05 m、Z 0.015 m、稳定窗口 500 ms、最少 3 个物理样本、证据最大年龄 500 ms、接触/位姿配对 100 ms、停止确认 5000 ms。运行命令：

```bash
python3 scripts/test_gazebo_scenarios.py \
  --scenario normal --scenario initial_pose_perturbation \
  --scenario placement_verification_reject --scenario post_grasp_drop \
  --scenario post_release_contact_feedback_loss --scenario cancel \
  --scenario timeout --scenario pause_resume --scenario time_reset \
  --output-dir .review-runs/pr11-8c36495-scenarios-20261004 \
  --process-timeout-s 240
```

场景矩阵退出码 **1**，9 个场景中 8 个满足预设预期，正常 seed 700 未成功，所以被正确标为失败。非零码没有被解释为整体通过：

| 场景 / seed / run_id | 实际结果 | 预期核对、资源和后续限制 |
|---|---|---|
| normal / 700 / `trial-001-seed-700-453629a46045` | `timed_out/TIMEOUT`，退出 1；发生在 `pick_object`，等待夹爪停止反馈；未通过物理审计 | 预期成功，故场景失败。停止被测量确认、资源为空；不能计成功。 |
| 初始位姿扰动 / 708 / `trial-002-seed-708-0f3e1fa85ec9` | `succeeded`，退出 0；扰动 `(+0.01,-0.01) m`，独立物理审计通过 | 预期成功、资源为空；场景通过。 |
| 放置验证拒绝 / 701 / `trial-003-seed-701-0cf90e05fe1a` | 放置运动已完成，测试随后把物体移离目标区；`failed/PLACEMENT_NOT_VERIFIED`，退出 1；运行时及独立审计均拒绝 | 明确发生在 `verify_placement`，停止确认且资源为空；场景通过。此项验证了动作完成但放置条件不满足时的拒绝。 |
| 抓取后掉落 / 702 / `trial-004-seed-702-8e038fe903c0` | 丢落注入命令成功；无假成功，独立审计拒绝；任务在 `place_object` 运动阶段以 `MOTION_FAILED` 结束，退出 1 | 预期禁止 success、要求审计拒绝且资源安全，符合预期。**没有到达结果验证阶段**，因此不能据此声称运行时放置验证器专门拒绝了掉落。 |
| 释放后接触反馈中断 / 703 / `trial-005-seed-703-3fec8a808a9b` | 释放确认后中断接触反馈；在 `verify_placement` 得到 `INVALID_VERIFICATION_EVIDENCE`，退出 1；运行时拒绝、资源为空 | 后验物理审计依据已经采集的完整物理样本判定物体确实稳定放置；但运行时缺少验证阶段所需的新鲜接触消息，因此任务仍拒绝。预设行为通过。 |
| 取消 / 704 / `trial-006-seed-704-9137b5baebb4` | `canceled/CANCELED`，请求被接受；取消请求至收到结果 0.268112 s，至确认停止 0.262319 s；停止确认后资源为空 | 两个时长分别按结果接收和反馈停止时刻记录；均小于 5 s，预设行为通过。 |
| 超时 / 705 / `trial-007-seed-705-4e5d2f73e34c` | 3 s 任务期限后 `timed_out/TIMEOUT`；从超时 deadline 至确认停止 0.059924 s；资源为空 | 小于 5 s，预设行为通过。 |
| 暂停/恢复 / 706 / `trial-008-seed-706-34c28cada9f5` | pause/unpause 均成功；任务成功且独立物理审计通过 | 暂停时缓存重复读数不增加稳定证据；恢复后使用新时间样本，预设行为通过。 |
| 仿真时间重置 / 707 / `trial-009-seed-707-2f6a5a237694` | reset 服务成功；控制器重启期间停止无法确认，`faulted/STOP_UNCONFIRMED`，未取得停止确认 | 手臂与夹爪资源均保留、禁止后续动作；符合 fail-closed 预设。不是“重置恢复成功”。 |

## 新固定种子批测

在被测代码 SHA `8c3649548b6b6b944838dd9c62aa01be9d1b4dd8` 和干净跟踪工作区执行，无重试：

```bash
python3 scripts/test_gazebo_trials.py --count 20 --seed-start 200 \
  --process-timeout-s 240 --output-dir .review-runs \
  --batch-id pr11-8c36495-20trials-20261004
```

正式批测返回 **1**，因为 18/20 达到所有成功门槛；进程没有假报整批通过。20 次均完成且有匹配 run id / seed / code SHA 的完整结果；无 JSON 无效或身份不符项，false success 为 0。

| 指标 | 实测 |
|---|---:|
| 完整运行 / seed | 20/20，200–219 |
| 独立验收成功 | 18/20（90%） |
| 任务/进程失败 | 2；均 `MOTION_FAILED`、退出码 1，资源已确认释放 |
| 独立审计失败 | 1（seed 207）；seed 202 的物理终态审计通过但任务失败，仍拒绝计成功 |
| 假成功 / 无效结果 | 0 / 0 |
| 全部 20 次 XY 误差 | 0.000626–0.015393 m，平均 0.004892 m |
| 被验收的 18 次 XY 误差 | 平均 0.004218 m，最大 0.009319 m |

失败详情：

- seed 202，run `trial-003-seed-202-4a42e11c5c2e`：MoveIt 的 `CheckStartStateCollision` 检出 `panda_leftfinger - sim_tray`，规划中止并以 `MOTION_FAILED` 结束。Gazebo 物体后来落在托盘，审计物理状态通过，但任务进程退出 1，所以不计成功。这是规划碰撞场景/场景边缘状态问题，不是审计误报。
- seed 207，run `trial-008-seed-207-734e526b7901`：控制器报告 Panda 旋转关节位置误差 `0.016722 rad` 和 `-0.002616 rad`，超过 `0.001 rad` 容差；没有确认释放、最终托盘接触或释放后新鲜无接触证据，独立审计拒绝。这是 DART/轨迹跟踪与控制器边界问题，尚未分离定位。

本轮 seed 控制 Gazebo 世界随机种子；MoveIt/OMPL 规划随机数未固定。同 seed 不承诺生成相同轨迹或结果。历史 seed 101 的失败及成功诊断重放、原始 19/20 结果仍在 [`phase2-gazebo.md`](../phase2-gazebo.md) 与原历史证据中，本轮没有替换或改写它们。该旧报告附加了单位和计时勘误。

## 证据和复核

- `evidence/batch-summary.json`：20 次正式批测机器可读汇总，含全部 seed、run_id、退出码、审计项及 XY 误差。
- `evidence/scenario-outcomes.json`：9 个指定场景的精简记录，包含预期、身份、执行步骤、独立审计、资源和计时字段。
- `evidence/logs/`：完整回归输出及当前批测失败和 9 个场景的 Gazebo 启动日志。
- `evidence/artifact-manifest.json`：已提交小型证据文件及本机原始结果 JSON/Gazebo 日志的大小和 SHA-256。原始逐采样结果保留在运行主机的 `.review-runs/`，合计 394,837,845 bytes；它们没有推送，也不能从新 clone 获取。本报告没有编造下载位置。逐采样文件过大且重复包含高频样本，因此 Git 中放入汇总、精简场景记录和关键日志；原始文件可在本机按 manifest 校验。

以下问题仍需独立审查：DART 对 Panda mimic 约束的警告、seed 700 的夹爪停止反馈超时、seed 202 的指尖/托盘起始碰撞、seed 207 的关节跟踪超差、seed 702 未进入验证阶段、世界重置时的 `STOP_UNCONFIRMED` fail-closed。这里的 18/20 仿真批测不是实机验证、安全认证或稳定性保证。场景物体位姿来自显式标记的 Gazebo 真值，不包含真实视觉；没有接入真实机器人、LLM/VLM/VLA SDK 或完整力控。
