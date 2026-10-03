# 技能目录与任务配置（5A）

6B 新增 `verify_grasp`、`verify_placement` 技能和 `verified_pick_place` 受审六步模板；目录版本仍为 1。ObjectLocator 依赖版本升为 2，验证技能依赖 ManipulationObserver；证据与迁移规则见 [感知证据与结果验证](perception_verification.md)。

## 查询入口

`/get_catalog` 使用 `robot_interfaces/srv/GetCatalog`，请求为空，响应 `schema_version=1`。

| 字段 | 含义 |
|---|---|
| `skills` | 技能 ID、描述、输入类型、文字条件与实现列表 |
| `implementations[].components` | 该实现所需的组件角色、接口类型和接口版本 |
| `exclusive_roles` / `execution_gate_role` | 该实现的独占角色与执行许可门 |
| `dependencies_satisfied` | 当前绑定的类型、版本和角色满足要求；不检查许可、物体状态或资源占用 |
| `unavailable_reason` | 依赖不满足的原因 |
| `tasks` | 配置任务名称、已安装模板、实现 ID 和必填参数 |
| `object_ids` / `target_ids` | 当前启动配置允许使用的对象和目标 |

查询不会构造技能、调用许可门、获取资源或下发动作。即使许可门关闭，类型/版本匹配的实现仍可显示 `dependencies_satisfied=true`。前置、维持和成功条件是契约说明，由技能实现检查，没有文字条件解释器。

技能输入当前只支持必填 `entity_id`，值必须符合 `[A-Za-z_][A-Za-z0-9_]*`，长度 1～64。核心 `Session` 和任务入口使用相同的输入校验；多余字段、缺失字段和非法标识符都会被拒绝。对象/目标角色在任务入口检查，不写死在模型或组件名称中。

## 配置对象、目标和任务别名

示例见 `src/robot_bringup/config/demo.yaml` 和 `src/robot_panda_demo/config/runtime.yaml`。

```yaml
robot_runtime:
  ros__parameters:
    base_frame: base_link
    object_ids: [part_a, part_b]
    target_ids: [bin_a, bin_b]
    entities.part_a.pose: [0.40, 0.10, 0.20, 0.0, 0.0, 0.0, 1.0]
    entities.part_b.pose: [0.30, -0.10, 0.20, 0.0, 0.0, 0.0, 1.0]
    entities.bin_a.pose: [0.60, -0.20, 0.15, 0.0, 0.0, 0.0, 1.0]
    entities.bin_b.pose: [0.50, 0.20, 0.30, 0.0, 0.0, 0.0, 1.0]
    task_names: [transfer, inspect]
    tasks.transfer.template_id: pick_place
    tasks.transfer.implementation_id: standard
    tasks.inspect.template_id: locate_object
    tasks.inspect.implementation_id: standard
```

位姿顺序为 `x,y,z,qx,qy,qz,qw`，坐标单位米，所有值必须为浮点数、有限，四元数需归一化。对象和目标共享 `base_frame`，ID 全局唯一。新增 ID 必须配置七个值。未提供参数文件时，保留 v0.3 的 `workpiece/tray` 默认位姿与任务请求。

这些位姿是示例机械臂动作目标，定位组件按调用时刻产生合成观测；不是相机测量，也不包含物体到抓取姿态的变换、接触几何或碰撞物体。真实感知需要另一个组件及相应技能实现，不能直接把检测物体中心当作机械臂目标。

保存为独立参数文件后启动：

```bash
ros2 launch robot_bringup demo.launch.py config_file:=/absolute/path/my_demo.yaml
ros2 service call /get_catalog robot_interfaces/srv/GetCatalog '{}'
ros2 action send_goal /execute_task robot_interfaces/action/ExecuteTask \
  "{task_name: transfer, object_id: part_b, target_id: bin_b, timeout_ms: 5000}" --feedback
ros2 action send_goal /execute_task robot_interfaces/action/ExecuteTask \
  "{task_name: inspect, object_id: part_b, target_id: '', timeout_ms: 5000}" --feedback
```

`base_frame`、实体列表/位姿、任务列表/模板/实现为只读启动参数。更新配置后重启示例；运行中通过参数服务修改会被拒绝。停止和故障恢复仍遵守执行规则，不能用重启替代实机停止确认。

## 模板与接收规则

| 已安装模板 | 输入与执行 |
|---|---|
| `pick_place` | `object_id` 必须是对象，`target_id` 必须是目标；定位对象、抓取、定位目标、放置 |
| `verified_pick_place` | 同上，抓取后加入 verify_grasp，释放后加入 verify_placement；依赖结果观察组件 |
| `locate_object` | `object_id` 可为任一配置实体，`target_id` 必须为空；只获取位姿 |

任务名称只是模板别名，同一别名可用于多组对象/目标。`implementation_id` 当前为模板内所有技能选用同名实现，默认 `standard`。新增实现时必须覆盖该模板的每种技能；逐步骤实现选择留给 5B 的结构化计划。

7A 增加 pose_resolved 同名实现集合；其中抓取和放置需要额外的 motion_target_resolver v1 组件。配置原生位姿时显式选择该实现，见 [位姿解析](target_resolution.md)。目录按实现 ID 描述可用性；客户端应按 ID 查找，不能假设数组第一项为 standard。

启动时验证重复 ID、位姿、模板白名单和实现依赖，加载已安装行为树。收到任务时在接收前校验名称、实体角色、1～600000 ms 的超时、每一步技能输入与组件绑定。后续步骤配置错误不会先执行前面的抓取动作。接收拒绝通过 Action 的 rejected 状态及运行时日志说明原因，当前没有增加结构化拒绝响应。

接收成功不保证物理执行成功。技能执行时仍检查许可、资源、观测时效、物体持有状态及测量反馈。抓取后故障可能保留物体持有状态；系统不会自动清空或盲目重试。

新模板需要添加受审的安装 XML、核心 `TaskCatalog` 的输入/步骤映射和对应测试。模板入口不接受客户端 XML 或脚本；5B 的结构化计划使用独立 `/execute_plan` 入口，见 [结构化技能计划](structured_plans.md)。

## C++ 迁移

`SkillDefinition::required_inputs` 改为 `inputs`，每项为 `{name, type, description}`。只有 `entity_id` 受支持，所有项必填；输入名重复、未知类型在注册时拒绝。`Skills::catalog(bindings)` 提供独立于 ROS 的目录；`validate_arguments` 和 `validate_dependencies` 可供任务入口复用。

`arm_motion/gripper` 组件接口仍为 v2；ROS `ExecuteTask` 字段不变。原有取消、超时、停止确认和资源规则保持有效。
