# 感知证据与结果验证（6B）

本阶段建立可替换的感知/结果证据契约，新增 `verify_grasp` 和 `verify_placement` 技能，并接入受审模板、结构化计划和执行记录。当前闭环使用显式合成证据；没有视觉检测、物体接触动力学或真实力传感器，不能用测试通过来证明已经抓稳实物。

后续 7A 已实现显式静态位姿解析组件及 pose_resolved 操作实现，见 [位姿解析契约与配置](target_resolution.md)。本页关于 6B 未实现目标转换的描述指当时阶段；真实标定求解与感知算法仍未实现。

## 技能与组件划分

| 层/对象 | 职责 | 当前示例 |
|---|---|---|
| ObjectLocator v2 组件 | 提供对象/目标位姿及证据来源、质量、时间、位姿语义 | ConfiguredDemoLocator |
| ManipulationObserver v1 组件 | 提供抓持/放置条件的技术证据；不决定任务是否成功，不下发动作 | DemoOutcomeObserver |
| locate_object 技能 | 检查位姿、来源、质量、时效后写入观测 | standard 实现 |
| verify_grasp 技能 | 判断对应抓取动作后，证据是否在指定窗口内持续满足抓持条件 | standard 实现 |
| verify_placement 技能 | 判断对应释放动作后，证据是否持续满足目标位置条件 | standard 实现 |
| verified_pick_place 任务 | 组织六个技能步骤 | 固定 BehaviorTree.CPP 模板 |

`ManipulationObserver` 不替代运动组件或控制器；MoveIt、ros2_control 继续处于运动/硬件链路。验证实现只依赖 `outcome` 观察组件，并通过 `motion`、`gripper` 角色取得本运行时的独占资源，避免本地任务在验证期间移动设备。没有执行许可门依赖，因为验证不发送动作；本地独占不覆盖外部控制源。

## 位姿证据

`Observation` 增加 `EvidenceMetadata{source, synthetic, quality}` 与 `PoseMeaning`。`source` 为 1～64 字符的组件来源 ID，遵循现有 ID 规则。quality 必须是有限的 [0,1] 来源特定分数；缺失值 -1 无效。当前最低接受分数为 0.8，**不是通用概率或跨模型可比较的精度**，真实来源需要规定分数定义及校准方法。

`EvidencePolicy` 默认不接受合成数据；本仓库示例 runtime 显式允许合成证据。未来实机组合不能直接复用这一许可。Locate 默认时效 2000 ms，验证证据默认时效 500 ms；零时间戳、未来时间、低质量、无来源或错误实体均不能形成有效结果。

| 位姿语义 | 可以做什么 |
|---|---|
| object_pose | 存储真实物体的原生感知姿态；不能直接下发给手臂 |
| motion_target | 经明确转换得到的末端执行器目标，可供当前 Manipulate 实现使用 |

配置示例的位姿均为 motion_target，来源为 configured_demo，synthetic=true。真实相机输出通常是 object_pose；运动前应通过显式、经标定的目标解析组件，将物体/抓取坐标、工具偏置、放置姿态和目标 frame 转换为 motion_target，并声明该实现的依赖。6B 未实现这一转换算法；若直接把 object_pose 交给现有抓取/放置实现，返回 `TARGET_RESOLUTION_REQUIRED`，不会发送运动命令。

## 抓稳与放置验收

`OutcomeEvidence` 包含对象 ID、放置目标 ID（抓稳时为空）、单调时间戳、valid、condition_met、来源/合成/质量和非零 sample_id。来源必须提供稳定的样本身份：重复读取同一传感器帧时 sample_id 不变，不能在查询时生成新编号或刷新旧帧时间。

验证流程为：

1. 抓稳要求世界状态持有对应对象且记录了测量抓取时间；放置要求存在匹配目标的释放候选和测量释放时间。
2. 仅使用严格晚于该动作效果时间的证据。仍新鲜但早于/等于动作时间的帧只使技能等待新证据。
3. 每份证据必须通过对象、目标、valid、来源、质量、时效和合成许可检查。
4. sample_id 与时间戳均严格推进才算新样本；相同 sample_id 仍是缓存帧，不受重复换算时间的微小抖动影响。来源或合成属性变化、新样本倒序、采样间隔超过证据时效时失败。当前条件为 false 时直接失败，不能把不稳定时段算入成功窗口。
5. 默认至少 3 份新样本、覆盖至少 100 ms 才成功；该窗口只是软件接受策略，真实抓稳还需来源提供力、滑移、触觉或其他明确物理条件。

验证只做非阻塞证据采样，没有重新抓取或重新移动。重复缓存帧不能积累成功样本；若没有新帧，证据过期或技能 deadline 会结束等待。取消立即终止只读采样，通过统一会话路径释放资源，不打开夹爪，也不伪造抓稳/放置结果。

放置验证成功才将候选提升到 known_locations，并保存带来源、质量、合成标记与样本窗口的 placement_verifications；失败保留候选。合成证明仍是合成证明，不能当作真实物体位置。开始新的相关操作会删除旧验证结果；开始放置会使抓稳证据失效。每个成功验证步骤在执行记录中单独保存证据，所以任务结束时仍能回溯抓稳与放置的验收。

常见错误码为 `INVALID_OBSERVATION`、`TARGET_RESOLUTION_REQUIRED`、`INVALID_VERIFICATION_EVIDENCE`、`GRASP_NOT_VERIFIED`、`PLACEMENT_NOT_VERIFIED`、`VERIFICATION_SOURCE_CHANGED`、`OUT_OF_ORDER_EVIDENCE`、`EVIDENCE_GAP`。失败不会自动重试动作。

## 运行示例

启动示例配置后提交六步任务：

```bash
ros2 launch robot_bringup demo.launch.py
ros2 action send_goal /execute_task robot_interfaces/action/ExecuteTask \
  "{task_name: verified_pick_place, object_id: workpiece, target_id: tray, timeout_ms: 5000}" --feedback
python3 scripts/inspect_runtime.py
```

结构化计划通过显式选项增加验证步骤；默认仍为原来的四步：

```bash
python3 scripts/plan_pick_place.py --verify-outcomes --timeout-ms 5000 --dry-run
python3 scripts/plan_pick_place.py --verify-outcomes --timeout-ms 5000
```

顺序为 locate_object → pick_object → verify_grasp → locate_object → place_object → verify_placement。全计划接收前验证操作顺序、实体角色和实现依赖，但不会创建预测观测或验证证据。计划仍使用 schema_version=1，支持五种当前技能、1～32 个顺序步骤。

Panda 示例使用相同任务/计划，timeout_ms 改为 90000。配置显式设置 `simulation_outcome_evidence: true`；其 DemoOutcomeObserver 只根据控制器/TF 的末端目标与合成夹持信号生成演示条件，**不读取 placement_candidates 来自证放置成功**，但也没有独立物体身份、滑移或落点检测。

| 启动只读参数 | 作用 |
|---|---|
| simulation_outcome_evidence | 注册合成观察组件；纯 mock 默认 true，panda_ros 默认 false、由示例 YAML 显式启用 |
| verification_window_ms | 稳定窗口，默认 100，范围 1～30000；必须与技能/任务 timeout 相容 |
| mock_verification_failure | none / grasp / placement；仅纯 mock 注入失败证据 |

关闭合成观察组件后，验证实现的目录依赖显示不可用，含验证的计划会在接收前拒绝；原四步计划仍可运行。若启动配置包含 verified_pick_place 模板但缺少观察组件，配置检查会阻止启动，需同时移除该任务别名。

## ROS 与已有组件迁移

- ObjectLocator 升为接口版本 2。旧 v1 适配器在绑定校验时拒绝；增加来源、质量、合成标记和位姿语义后再声明 v2。
- ArmMotion/Gripper 保持版本 2。反馈新增 sample_id；Panda 夹爪使用原始 JointState 时间戳纳秒，手臂使用 JointState/TF 中较旧的源时间戳纳秒。演示观察组件以这些同一 ROS 时钟来源中最旧的身份作为组合样本身份，最旧输入仍为缓存帧时，其他输入更新不能刷新组合身份。重复轮询同一 ROS 消息不会产生新样本。独立真实 ManipulationObserver 可使用其自己的传感器序列，不依赖这些演示字段。
- GetRuntimeState/GetExecution 诊断 schema 升为 2；ObservationSnapshot 增加证据/语义字段，WorldSnapshot 增加验证数组，SkillExecutionRecord 增加 has_verification 与证据。ROS 消息发生变化，须重新构建所有包并使用一致 overlay，不能混用旧生成接口。
- GetCatalog、ExecuteTask、ExecutePlan 的协议版本/请求字段保持原样。inspect_runtime.py 当前接受诊断 schema 2；旧备份使用对应版本客户端。

## 下一步接真实来源

先选择设备与传感器，明确时间同步、frame、标定、位姿到运动目标转换、样本身份、质量定义和失败语义，再实现真实 ObjectLocator 或 ManipulationObserver。抓稳证据应覆盖滑移/负载等目标条件；放置证据应独立观测对应物体与目标位置。分别验证缺帧、低质量、身份错误、遮挡、对象丢失、取消和超时，保持停止确认与资源锁定规则。力控和 VLA 连续动作仍需独立动作策略/控制契约，本阶段未加入具体模型 SDK 或硬件驱动。
