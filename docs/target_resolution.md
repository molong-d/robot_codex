# 显式位姿解析（7A）

本阶段把 `object_pose` 到末端 `motion_target` 的几何转换做成可替换组件，并以 `pose_resolved` 第二种技能实现接入已有任务、结构化计划和停止确认流程。没有引入真实相机、标定求解器、抓取规划算法或特定模型 SDK；提供的位姿和标定均为合成演示数据。

## 技能与组件边界

| 对象 | 职责 |
|---|---|
| ObjectLocator v2 | 提供指定物体或放置目标的原生位姿、坐标、捕获时间及来源质量 |
| MotionTargetResolver v1 | 根据用途和明确标定，把原生位姿转换为所控制工具坐标的目标位姿 |
| StaticMotionTargetResolver | 一个源坐标、基坐标和工具组合；使用固定外参与两个工具偏置 |
| pick_object / place_object 的 pose_resolved 实现 | 检查原始证据、解析输出和工具一致性，组织运动、夹爪与结果验收 |
| verify_grasp / verify_placement | 使用独立结果证据验收，目标解析不能自证实物抓稳或放置成功 |

`standard` 使用已经明确标为 motion_target 的基坐标位姿；`pose_resolved` 只使用 object_pose，并增加 `target_resolver` 角色依赖。两种实现共享输入、抓持/释放结果语义、执行许可和手臂/夹爪独占规则。解析器没有下发动作的方法，也不求解 IK、生成接近路径或执行控制。

任务别名当前为所有步骤选择同名实现。因此定位和验证步骤也注册了 pose_resolved，它们复用原来的只读行为，**只有抓取和放置步骤增加解析依赖**。结构化计划可逐步选择实现；不能给同一末端目标重复叠加工具偏置。

## 变换约定

数组均为 `[x, y, z, qx, qy, qz, qw]`，平移单位为米，四元数为有效单位旋转。用 `T_A_B` 表示 B 坐标在 A 坐标中的位姿：

```text
T_base_tool = T_base_source × T_source_entity × T_entity_tool
```

`source_in_base` 为 T_base_source，不是其逆变换。抓取时 entity 为物体，使用 `grasp_tool_offset`；放置时 entity 为放置目标，使用 `place_tool_offset`。偏置平移随实体旋转，不能直接将三个平移向量相加。相机内参、深度尺度、手眼标定求解和物体抓取点选择不在该公式实现内。

工具坐标必须与运动适配器的 `end_effector_link` 一致。这里的偏置已经包含要控制的工具到实体的关系；若 MoveIt 控制法兰而外部配置针对 TCP，必须由明确配置转换后才能使用，不能默认两者相同。

StaticCalibration 在创建时验证 ID、坐标名称与三个刚体变换，构造后不可修改。它只支持静态外参；移动相机或动态坐标应实现另一种 MotionTargetResolver，并明确采样时刻的变换有效性，不能将这个固定外参类当作最新 TF 查询。

## 证据与失败规则

技能先验证原始位姿 ID、valid、姿态、捕获时间、来源质量和合成许可，再调用解析器。输出必须保持原始 ID、捕获时间、来源与来源质量，声明正确基坐标、工具、motion_target 语义及有效标定 ID。输入为合成证据时，输出不能去掉该标记；合成标定作用于真实输入时，结果同样为合成数据，并再次按技能策略检查。

质量字段仍是原感知来源的分数，**不表示转换后目标的整体精度**。真实标定误差、工具误差与抓取点正确性需由实际接入方案另行规定。转换不能把旧观测刷新为当前时间。

| 错误码 | 触发 |
|---|---|
| STALE_OBSERVATION | 原始位姿缺失、无效、过期或来源质量不满足 |
| TARGET_RESOLUTION_REQUIRED | standard 收到了同基坐标下尚未解析的 object_pose |
| INVALID_SOURCE_POSE | pose_resolved 收到了已经解析的 motion_target |
| TARGET_RESOLUTION_FAILED | 源坐标不匹配、解析不可用/异常、错误工具或输出改变了证据 |

上述错误在手臂命令前返回，不消耗有效原始观测、不创建放置候选，也不自动重试。缺少解析组件时，整个结构化计划在准入阶段被拒绝。开始运动后使用原来的取消、超时和测量停止确认流程；停止未确认时仍持有资源。

## 运行配置

纯 mock 配置：

```bash
ros2 launch robot_bringup demo.launch.py \
  config_file:=$(ros2 pkg prefix robot_bringup)/share/robot_bringup/config/demo_native_poses.yaml
```

Panda GenericSystem 配置：

```bash
ros2 launch robot_panda_demo panda.launch.py \
  config_file:=$(ros2 pkg prefix robot_panda_demo)/share/robot_panda_demo/config/runtime_native_poses.yaml
```

启动完成后另开终端，source 一致的 ROS 与 overlay，提交六步计划：

```bash
python3 scripts/plan_pick_place.py --implementation pose_resolved --verify-outcomes --timeout-ms 5000
```

Panda 将 timeout-ms 改为 90000，并等待控制器激活与新鲜关节反馈。配置任务别名也已选择 pose_resolved，可直接提交 verified_pick_place。

| 启动只读配置 | 含义 |
|---|---|
| perception_mode | motion_target 或 object_pose；默认前者 |
| perception_frame | 原生观测所属坐标；object_pose 模式必填 |
| native_entities.ID.pose | 原生位姿；每个已配置对象/目标均须显式提供 |
| entities.ID.pose | 独立的示例预期末端目标，仍供合成结果观察组件使用 |
| target_resolution.enabled | 是否注册解析组件；默认 false |
| target_resolution.calibration_id | 标定版本 ID，非空且遵循现有 ID 规则 |
| target_resolution.source_frame | 标定对应的源坐标名称 |
| target_resolution.tool_frame | 必须匹配 end_effector_link |
| target_resolution.source_in_base | 源坐标在基坐标中的静态外参；显式 7 元数组 |
| target_resolution.grasp_tool_offset | 工具在物体坐标中的抓取偏置；显式 7 元数组 |
| target_resolution.place_tool_offset | 工具在目标坐标中的放置偏置；显式 7 元数组 |
| target_resolution.synthetic | 标定是否合成；默认 true，示例显式为 true |

启用解析时没有隐式单位外参或零工具偏置；缺失数组、无效四元数和工具不一致会阻止启动。原生位姿与预期末端目标分别配置，运行时不会从预期末端目标逆算感知结果。两种数据仍是固定的合成样本，不是相机测量。

## 记录、兼容与下一步

执行记录保留实际 implementation_id；操作技能第一次 running 状态的 message 包含 calibration ID、源坐标和工具名称。原生观测保持 object_pose 语义，定位不会将其改写成末端目标。当前没有新增结构化解析快照，执行记录不保存完整外参、工具偏置或已消费的输入位姿；复现实验需同时保存对应版本的启动配置。

ROS 消息、Action 请求、目录/计划版本和诊断 schema 2 均未改变。默认配置继续使用 standard，额外实现由目录按 ID 查询，客户端不能假设 implementations[0] 是默认实现。修改过的 C++ 包和安装配置仍需重新构建并 source 一致 overlay。

下一步选择真实感知设备和抓取策略后，将原生位姿、采样时刻的变换、标定版本与物体相关的工具偏置接到上述契约。当前一个固定抓取偏置和一个固定放置偏置适用于共享工具定义的示例，不是通用抓取规划器。真实结果验收继续由 ManipulationObserver 及验证技能承担；VLA 可注册另一种技能实现，独立声明动作策略等依赖，不必经过这个几何解析器。
