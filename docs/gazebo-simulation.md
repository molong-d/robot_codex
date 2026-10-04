# Panda Gazebo Harmonic 抓取与放置

此入口增加独立的 Gazebo Harmonic 仿真，不改变原有 `robot_panda_demo` GenericSystem 测试。场景运行 DART 物理引擎，包含 Panda、平行夹爪、桌面、动态方块和托盘。控制路径是 MoveIt 规划、`gz_ros2_control`、ros2_control 控制器及运行时已有的 `pick_place` 技能。

## 环境与启动

宿主机需要 Docker Engine、可读写 Docker daemon 的当前用户、Git 和足够的磁盘空间来构建镜像与 ROS 工作区。项目在宿主机已有 ROS Humble/Noetic 的 Ubuntu 22.04.5 上实际使用隔离的 Ubuntu 24.04.5 / ROS 2 Jazzy 容器；没有覆盖宿主机 ROS。CPU/headless 可运行，不需要 GPU 或桌面显示。固定基础镜像、依赖来源和实测包版本见[环境说明](environment.md)与[阶段二报告](reports/phase2-gazebo.md)。

在仓库根目录运行：

```bash
scripts/with_jazzy.sh bash scripts/test_all_jazzy.sh
```

该脚本完成核心测试、全工作区构建、colcon 测试和 ROS/Panda 集成测试。仿真包也包含在全工作区构建中。构建后启动独立仿真：

```bash
scripts/with_jazzy.sh scripts/run_panda_gz.sh
```

仿真不依赖手动启动 runtime；launch 会启动 Gazebo、桥接、MoveIt、控制器、场景同步器和任务 runtime。用自动创建唯一输出和身份的正常场景复现任务：

```bash
scripts/with_jazzy.sh python3 scripts/test_gazebo_scenarios.py --scenario normal
```

单次测试须使用新的 `--result-json` 和 `--launch-log` 路径；已有结果文件会被拒绝。故障场景脚本会自动创建批次目录、唯一 run_id 和独立 JSON/日志。

## 场景与成功判定

Gazebo 的 `PosePublisher` 提供明确标记为 `synthetic: true` 的仿真真值位姿；接触传感器提供物体与夹爪/支撑面的碰撞证据。`gz_scene_sync.py` 只用新时间戳更新 MoveIt 碰撞场景；检测到物体和左右指同时接触时才附着规划场景物体。夹持消息携带 Gazebo 源时间戳，消费者拒绝重复、过期和未来样本；Gazebo 中的物体始终由接触动力学移动，代码不会瞬移物体。技能通过 Gazebo 物体位姿证据验证物体抬升、释放落点和稳定性；固定种子测试工具还会独立检查实际托盘底板接触。

物理验收阈值的唯一配置位于 `src/robot_panda_gz_sim/config/acceptance.json`，launch 会将同一组值传给运行时、场景同步器和独立审计器。默认 XY 径向误差不超过 0.05 m，Z 误差不超过 0.015 m，停止确认超时 5 s，证据最大年龄 500 ms，稳定窗口 500 ms 且至少 3 个物理样本。对象速度和稳定时长按 Gazebo 源仿真时间计算；消息新鲜度、服务响应超时和运动停止超时按单调墙钟计算。暂停期间重复缓存样本不增加样本 ID 或稳定时长。仅在 ROS 仿真时钟明确回退超过 100 ms 时开始新 epoch；普通乱序源时间戳会被拒绝。回退后旧 epoch 的位姿、关节和接触缓存失效，旧样本在跨过重置前水位线前不接受。

独立审计不读取运行时 `success` 布尔值。批测只有在子进程退出码为 0、结果身份/run_id/seed/代码 SHA 一致、任务成功、独立物理审计通过，且执行记录确认停止和资源释放时才计为成功。物理审计核验同一接触采样中的左右指接触、接触后的物体抬升、释放事件后的新鲜无指接触、最终新鲜托盘底接触、坐标系/epoch/物体身份、位置误差，以及整个仿真时间稳定窗口。历史接触不能代替末态接触。

## 扰动与故障场景

`scripts/test_gazebo_scenarios.py` 使用固定场景配置运行故障验收：

```bash
scripts/with_jazzy.sh python3 scripts/test_gazebo_scenarios.py --scenario placement_verification_reject
scripts/with_jazzy.sh python3 scripts/test_gazebo_scenarios.py --scenario post_grasp_drop
scripts/with_jazzy.sh python3 scripts/test_gazebo_scenarios.py --scenario post_release_contact_feedback_loss
scripts/with_jazzy.sh python3 scripts/test_gazebo_scenarios.py --scenario cancel --scenario timeout
scripts/with_jazzy.sh python3 scripts/test_gazebo_scenarios.py --scenario pause_resume --scenario time_reset
```

验收期望状态、错误码、必须到达的技能阶段、停止确认及资源保留条件预先写在 `scenario_expectations.json`。放错场景要求 `place_object` 成功且 `verify_placement` 明确拒绝；seed 48 的旧 MOTION_FAILED 记录不作为此项证据。掉落场景通过动作打开真实模拟夹爪触发，不移动物体坐标。接触反馈中断在 release 已确认之后通过参数停止发布。非零退出码本身不构成故障测试通过。结果与检查报告分别保存于新的 `.review-runs/scenarios-<batch-id>/` 子目录。测试会为每次仿真重新启动完整 stack；不要并行运行多个场景，它们共享 ROS domain 和 Gazebo 服务名。

固定种子重复运行（默认 20 次，默认 seed 从 200 开始），逐次使用新 run_id 和独立目录，设置 240 s 子进程墙钟超时并保留诊断：

```bash
scripts/with_jazzy.sh python3 scripts/test_gazebo_trials.py --count 20 --seed-start 200
```

每批会拒绝已存在的 batch-id，JSON 和日志写入唯一 `.review-runs/<batch-id>/`。可用 `--output-dir` 和 `--batch-id` 指定新位置/名称；脚本拒绝复用旧目录。批测不重试失败场景；汇总中的通过数、终端显示和进程退出码使用相同的验收布尔值。

取消时间分别记录取消请求、动作结果收讫和独立关节停止确认。`cancel_to_result_s` 在结果回调时冻结，不含其后的 0.5 s 观测窗口；如果关节样本重复、过期或停止未能确认，停止时间记为未测量。

`--seed` 传递给 Gazebo Sim，用于固定世界随机种子；当前 MoveIt/OMPL 规划随机数流未配置固定 seed。因此固定 Gazebo seed 不保证生成相同机械臂轨迹。一次 seed 101 运行失败、同 seed 独立重放成功的实测和原始证据见[阶段二报告](reports/phase2-gazebo.md)。

## 边界与已知风险

- 位姿来自仿真真值，不是相机或视觉算法。仿真和真值插件只用于隔离验证。
- Gazebo DART 在启动时报告不支持 URDF mimic constraint。当前控制路径驱动 Panda 手指并观察到左右指接触与物体抬升，但夹爪联动与接触模型仍需复核，重复场景数据用于量化这一风险。
- 仿真暂停/恢复已做实测；进程重启、系统重置及真实机器人时钟行为不等同于通过硬件安全验证。
- 未接真实机器人、具体 LLM/VLM/VLA SDK、视觉算法或完整力控系统。Gazebo 成功不能作为实机能力或安全认证。
