# robot_codex

面向可扩展机器人应用的初步框架：**ROS 2 集成 + BehaviorTree.CPP 执行 + 技能语义 + 可替换组件**。

当前版本为 **v0.4 技能目录、结构化计划与执行记录开发版（5A/5B/6A）**。包含 ROS 2 Action 服务、技能目录查询、可配置对象/目标/任务别名、结构化技能计划、有界执行记录及状态查询、BehaviorTree.CPP 行为树，以及可独立测试的 C++ 核心。位姿合法性、位置/姿态容差、反馈时效与停止确认均参与技能验收。尚未连接真实机器人。

## 技能与组件

| 概念 | 职责 | 示例 |
|---|---|---|
| 任务 | 组织完整目标 | 定位、抓取并放到托盘 |
| 技能 | 对目标行为及结果负责 | locate_object、pick_object、place_object |
| 组件 | 实现可复用技术功能 | 位姿感知、手臂运动、夹爪、动作策略、控制 |
| 适配器 | 对接设备或第三方框架 | 厂商 SDK、MoveIt、ros2_control |

技能定义与技能实现分离；实现通过角色绑定获取组件。替换 `mock_arm` 为 `slow_mock_arm` 不需要修改技能或行为树。这里的“组件”不等于 ROS 2 Composable Node。

## 包结构

| 路径 | 内容 |
|---|---|
| `src/robot_core` | 无 ROS 依赖的 C++17 契约、注册、绑定、资源管理、技能会话；mock 示例 |
| `src/robot_interfaces` | 任务/计划 Action、技能目录、运行状态与执行记录服务及消息 |
| `src/robot_bt_runtime` | BehaviorTree.CPP 4 执行引擎与 ROS 2 Action 服务 |
| `src/robot_bringup` | 启动文件及组件选择配置 |
| `src/robot_ros_adapters` | 非阻塞 MoveIt/并联夹爪 Action 客户端、JointState/TF 反馈 |
| `src/robot_panda_demo` | Panda + ros2_control GenericSystem 示例及配置 |
| `docs` | 架构、扩展流程、执行约束和后续路线 |
| `scripts` | 核心测试与 ROS 2 集成测试 |

## 无 ROS 环境先运行核心测试

需要 Bash 和支持 C++17 的 g++：

```bash
bash scripts/test_core.sh
```

此命令验证技能闭环、组件替换、输入/绑定校验、观测时效、资源冲突、取消确认、超时、故障锁定等。它不代替 ROS 2 构建与实机测试。

## ROS 2 构建与运行

首个构建目标为 **Ubuntu 24.04 + ROS 2 Jazzy + BehaviorTree.CPP 4**。使用已有 ROS 2 Jazzy 环境，在仓库根目录运行：

```bash
source /opt/ros/jazzy/setup.bash
rosdep update --rosdistro jazzy
rosdep install --from-paths src --ignore-src --rosdistro jazzy -r -y
colcon build --symlink-install --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
ros2 launch robot_bringup demo.launch.py
```

另开终端，在仓库根目录 source 环境并提交 mock 任务：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 action send_goal /execute_task robot_interfaces/action/ExecuteTask \
  "{task_name: pick_place, object_id: workpiece, target_id: tray, timeout_ms: 5000}" --feedback
```

预期任务状态 `succeeded`、`success: true`。只接受启动时配置的对象、目标和已安装任务模板的别名。原有 Action 字段和默认请求保持兼容。

查询技能、输入类型、实现依赖和任务目录：

```bash
ros2 service call /get_catalog robot_interfaces/srv/GetCatalog '{}'
```

使用示例配置中的第二组对象和目标：

```bash
ros2 action send_goal /execute_task robot_interfaces/action/ExecuteTask \
  "{task_name: transfer_two, object_id: workpiece_two, target_id: tray_two, timeout_ms: 5000}" --feedback
```

`transfer_two` 是 `pick_place` 模板的配置别名，不绑定特定对象。只读定位任务 `inspect_object` 使用 `locate_object` 模板，`target_id` 必须为空。启动配置、校验规则及目录字段含义见 [技能目录与任务配置](docs/task_catalog.md)。

## 结构化计划与规划示例

运行时还提供 `/execute_plan`。示例规划客户端先查询目录，将目标分解为技能步骤，然后通过同一个执行引擎运行：

```bash
python3 scripts/plan_pick_place.py --object workpiece_two --target tray_two --timeout-ms 5000 --dry-run
python3 scripts/plan_pick_place.py --object workpiece_two --target tray_two --timeout-ms 5000
```

Panda 示例使用 `--timeout-ms 90000`。这是一种确定性任务分解，不是通用符号规划器或模型调用。计划必须通过全步骤参数、实现依赖、实体角色及操作顺序校验；两种 Action 入口共用忙碌互斥、取消、超时和停止确认。协议与后续模型接入方式见 [结构化计划](docs/structured_plans.md)。

替换组件（先停止原有 mock 节点）：

```bash
ros2 launch robot_bringup demo.launch.py motion_component:=slow_mock_arm
```

模拟抓取失败：

```bash
ros2 launch robot_bringup demo.launch.py mock_fail_pick:=true
```

预期 `GRIPPER_FAILED`，不会执行后续定位托盘与放置技能。模拟执行许可拒绝：

```bash
ros2 launch robot_bringup demo.launch.py mock_motion_permitted:=false
```

此时抓取返回 `SAFETY_INTERLOCK`，不会取得手臂/夹爪资源或下发动作。参数 `mock_action_ticks` 可增大动作持续时间，方便观察取消和超时；这只是计次 mock，不是物理仿真。

## 执行记录与状态查询

查看当前执行、资源占用、示例世界状态和最近任务记录，或导出诊断 JSON：

```bash
python3 scripts/inspect_runtime.py
python3 scripts/inspect_runtime.py --output execution-snapshot.json
```

任务可按 Action goal UUID 查询；记录包含实际技能结果与状态变化，并区分观测事实和推断放置位置。内存历史默认保留 32 个完成任务，不支持跨重启恢复或自动重试。接口与字段语义见 [执行记录](docs/execution_records.md)。

## 完整验证

停止手动启动的 runtime，source 上述环境后运行：

```bash
colcon test --event-handlers console_direct+ --return-code-on-test-failure
colcon test-result --verbose
python3 scripts/test_ros.py
python3 scripts/test_panda.py
```

ROS 集成测试会自行启动/停止节点，验证实际行为树及 Action 的成功、目录查询、第二组对象/目标、配置别名、非法配置/参数拒绝、组件替换、运动/夹爪失败、执行许可拒绝、超时、忙碌拒绝和取消确认。GitHub Actions 配置运行核心与 ROS 两类测试，结果见仓库 Actions 页面。

## Panda ROS 后端示例

停止 mock runtime 后运行：

```bash
ros2 launch robot_panda_demo panda.launch.py
```

另开终端，source ROS 与本仓库 overlay，等待 MoveIt 和控制器就绪后发送：

```bash
ros2 action send_goal /execute_task robot_interfaces/action/ExecuteTask \
  "{task_name: pick_place, object_id: workpiece, target_id: tray, timeout_ms: 90000}" --feedback
```

此示例实际运行 MoveIt 规划、轨迹控制器、夹爪控制器和 TF，硬件固定为 GenericSystem。目标位姿为示例配置，夹持信号为显式合成信号；没有物体接触物理，也没有视觉检测。详细配置、停止语义与实机接入要求见 [ROS 后端](docs/ros_backend.md)。

## 当前边界

- 单机器人、单活动任务、单线程 executor；没有分布式资源锁。
- 定位示例返回固定 mock 位姿、坐标系与时间戳，**不是 6D 感知算法**。
- 条件描述用于契约说明；当前条件由技能 C++ 实现检查，没有通用谓词解释器或自动规划器。
- 技能/组件通过显式工厂注册，不支持运行时下载插件；扩展实际第三方适配器时可再接 pluginlib 或 ROS Action 客户端。
- 任务模板是手写行为树；LLM/VLM 任务规划和 VLA 动作策略只预留架构位置，没有模型调用或 API key 要求。
- 输入类型仅为必填 `entity_id`；对象/目标角色由任务模板或计划校验器检查。结构化计划 v1 支持三种现有技能、1～32 个顺序步骤；没有通用位姿、力、轨迹参数、分支或自动重试。
- 停止未确认或异常时保留资源并拒绝后续任务；mock 节点可重启复位。真实设备必须先确认物理状态，不能将进程重启当作停机确认。
- 初始仓库未指定开源许可；本次未替仓库所有者授予开源许可证。包清单使用 `LicenseRef-Proprietary` 占位，发布前由所有者选择许可证并同步修改。

阅读：[架构与术语](docs/architecture.md) · [技能目录与任务配置](docs/task_catalog.md) · [结构化计划](docs/structured_plans.md) · [执行记录](docs/execution_records.md) · [扩展指南](docs/extensions.md) · [执行规则](docs/execution.md) · [路线与验证状态](docs/roadmap.md)

稳定版本备份和升级顺序见 [版本与回退](docs/versions.md)。组件契约位于 `components.hpp`，几何验证位于 `geometry.hpp`，技能实现位于 `skills.hpp`，mock 后端保留在 `demo.hpp`。
