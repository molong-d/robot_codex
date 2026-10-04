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

仿真不依赖手动启动 runtime；launch 会启动 Gazebo、桥接、MoveIt、控制器、场景同步器和任务 runtime。任务验证使用相同容器环境，在另一个终端运行：

```bash
scripts/with_jazzy.sh scripts/test_gazebo.sh --seed 42 --timeout-ms 30000
```

测试工具保存 JSON 结果和启动日志到 `docs/reports/`。若测试脚本创建前缺少 build/install，先运行上面的完整构建命令。

## 场景与成功判定

Gazebo 的 `PosePublisher` 提供明确标记为 `synthetic: true` 的仿真真值位姿；接触传感器提供物体与夹爪/支撑面的碰撞证据。`gz_scene_sync.py` 只用新时间戳更新 MoveIt 碰撞场景；检测到物体和左右指同时接触时才附着规划场景物体。夹持消息携带 Gazebo 源时间戳，消费者拒绝重复、过期和未来样本；Gazebo 中的物体始终由接触动力学移动，代码不会瞬移物体。技能通过 Gazebo 物体位姿证据验证物体抬升、释放落点和稳定性；固定种子测试工具还会独立检查实际托盘底板接触。

固定验收参数位于 `src/robot_panda_gz_sim/config/runtime.yaml`：平面放置误差不超过 0.05 m、Z 误差不超过 0.015 m、停止确认超时 5 s、任务超时 30 s、稳定观测窗口 500 ms。窗口只接受推进的仿真时间样本；暂停期间重复缓存消息不会增加证据。恢复或时间倒退会清除接触/位姿缓存并开始新样本 epoch。C++ 和 Python 层都检查仿真源时间是否落在当前 ROS 仿真时钟的 500 ms 窗口内。

单场景结果还会保存在结构化 JSON 中。固定种子测试工具只有在运行时成功且独立终态复核满足以下条件时才计为通过：两侧夹爪均曾与物体接触、物体最终释放、Gazebo 报告托盘底板接触，且物体终态位姿在配置误差范围内。运行时技能会读取独立 Gazebo 位姿证据，要求抓取后物体抬升，并在放置后于配置的 500 ms 窗口内稳定；固定种子审计额外记录托盘底板接触，发现技能误报成功时单独计为 false success。

## 扰动与故障场景

`scripts/test_gazebo.sh` 将参数传给 `scripts/test_gazebo.py`。例如：

```bash
scripts/with_jazzy.sh scripts/test_gazebo.sh --seed 43 --object-dx 0.015 --object-dy -0.010
scripts/with_jazzy.sh scripts/test_gazebo.sh --seed 49 --grasp-miss-x 0.08
scripts/with_jazzy.sh scripts/test_gazebo.sh --seed 44 --pause-after-ms 2000 --pause-duration-ms 1000
scripts/with_jazzy.sh scripts/test_gazebo.sh --seed 50 --reset-after-ms 1500 --timeout-ms 5000
scripts/with_jazzy.sh scripts/test_gazebo.sh --seed 45 --cancel-after-ms 1500
scripts/with_jazzy.sh scripts/test_gazebo.sh --seed 46 --timeout-ms 3000
scripts/with_jazzy.sh scripts/test_gazebo.sh --seed 47 --drop-grasp-feedback --timeout-ms 3000
scripts/with_jazzy.sh scripts/test_gazebo.sh --seed 48 --placement-offset-y 0.08
```

`--grasp-miss-x` 注入夹取位姿偏置，验证夹空后不会报告抓取成功；`--reset-after-ms` 在任务中重置 Gazebo 世界并检查计时回退后的反馈处理。结果状态、目标接受/取消、任务反馈、最后物体位姿、接触、关节状态和停止时间均写入 JSON。启动日志用于区分启动/配置问题、MoveIt 规划错误与物理接触结果。测试会为每次仿真重新启动完整 stack；不要并行运行多个场景，它们共享 ROS domain 和 Gazebo 服务名。

固定种子重复运行（默认 20 次），逐次保留场景日志与结构化结果，并生成独立终态审计汇总：

```bash
scripts/with_jazzy.sh python3 scripts/test_gazebo_trials.py --count 20 --seed-start 100
```

可用 `--output-dir` 和 `--summary` 指定报告输出位置。重复测试不会重试失败场景；汇总中的成功率和误报成功数按实际结果计算。

`--seed` 传递给 Gazebo Sim，用于固定世界随机种子；当前 MoveIt/OMPL 规划随机数流未配置固定 seed。因此固定 Gazebo seed 不保证生成相同机械臂轨迹。一次 seed 101 运行失败、同 seed 独立重放成功的实测和原始证据见[阶段二报告](reports/phase2-gazebo.md)。

## 边界与已知风险

- 位姿来自仿真真值，不是相机或视觉算法。仿真和真值插件只用于隔离验证。
- Gazebo DART 在启动时报告不支持 URDF mimic constraint。当前控制路径驱动 Panda 手指并观察到左右指接触与物体抬升，但夹爪联动与接触模型仍需复核，重复场景数据用于量化这一风险。
- 仿真暂停/恢复已做实测；进程重启、系统重置及真实机器人时钟行为不等同于通过硬件安全验证。
- 未接真实机器人、具体 LLM/VLM/VLA SDK、视觉算法或完整力控系统。Gazebo 成功不能作为实机能力或安全认证。
