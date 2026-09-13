# robot_codex

面向可扩展机器人应用的初步框架：**ROS 2 集成 + BehaviorTree.CPP 执行 + 技能语义 + 可替换组件**。

当前版本为 **v0.1 mock 骨架**。包含实际的 ROS 2 Action 服务、BehaviorTree.CPP 行为树，以及可独立测试的 C++ 核心。没有连接真实机器人，也没有实现 MoveIt、ros2_control、6D 感知、力控或具体大模型适配。

## 技能与组件

| 概念 | 职责 | 示例 |
|---|---|---|
| 任务 | 组织完整目标 | 定位、抓取并放到托盘 |
| 技能 | 对目标行为及结果负责 | locate_object、pick_object、place_object |
| 组件 | 实现可复用技术功能 | 感知、运动规划、动作策略、控制 |
| 适配器 | 对接设备或第三方框架 | 厂商 SDK、MoveIt、ros2_control |

技能定义与技能实现分离；实现通过角色绑定获取组件。替换 `mock_arm` 为 `slow_mock_arm` 不需要修改技能或行为树。这里的“组件”不等于 ROS 2 Composable Node。

## 包结构

| 路径 | 内容 |
|---|---|
| `src/robot_core` | 无 ROS 依赖的 C++17 契约、注册、绑定、资源管理、技能会话；mock 示例 |
| `src/robot_interfaces` | `ExecuteTask.action`，类型明确的任务入口 |
| `src/robot_bt_runtime` | BehaviorTree.CPP 4 执行引擎与 ROS 2 Action 服务 |
| `src/robot_bringup` | 启动文件及组件选择配置 |
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

预期任务状态 `succeeded`、`success: true`。v0 只接受此已安装任务模板及示例物体/目标；不会执行外部任意 XML、脚本或模型生成代码。

替换组件（先停止原有 mock 节点）：

```bash
ros2 launch robot_bringup demo.launch.py motion_component:=slow_mock_arm
```

模拟抓取失败：

```bash
ros2 launch robot_bringup demo.launch.py mock_fail_pick:=true
```

预期 `EXECUTION_FAILED`，不会执行后续放置技能。参数 `mock_action_ticks` 可增大动作持续时间，方便观察取消和超时；这只是计次 mock，不是物理仿真。

## 完整验证

停止手动启动的 runtime，source 上述环境后运行：

```bash
colcon test --event-handlers console_direct+ --return-code-on-test-failure
colcon test-result --verbose
python3 scripts/test_ros.py
```

ROS 集成测试会自行启动/停止节点，验证实际行为树及 Action 的成功、组件替换、失败、未知任务拒绝、超时、忙碌拒绝和取消确认。GitHub Actions 配置运行核心与 ROS 两类测试，结果见仓库 Actions 页面。

## 当前边界

- 单机器人、单活动任务、单线程 executor；没有分布式资源锁。
- 定位示例只提供对象 ID、坐标系与时间戳，**不是 6D 感知算法**。
- 条件描述用于契约说明；当前条件由技能 C++ 实现检查，没有通用谓词解释器或自动规划器。
- 技能/组件通过显式工厂注册，不支持运行时下载插件；扩展实际第三方适配器时可再接 pluginlib 或 ROS Action 客户端。
- 任务模板是手写行为树；LLM/VLM 任务规划和 VLA 动作策略只预留架构位置，没有模型调用或 API key 要求。
- 停止未确认或异常时保留资源并拒绝后续任务；mock 节点可重启复位。真实设备必须先确认物理状态，不能将进程重启当作停机确认。
- 初始仓库未指定开源许可；本次未替仓库所有者授予开源许可证。包清单使用 `LicenseRef-Proprietary` 占位，发布前由所有者选择许可证并同步修改。

阅读：[架构与术语](docs/architecture.md) · [扩展指南](docs/extensions.md) · [执行规则](docs/execution.md) · [路线与验证状态](docs/roadmap.md)
