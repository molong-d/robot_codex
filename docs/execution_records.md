# 执行记录与状态查询（6A）

本阶段增加只读诊断：查询当前执行、按 Action goal UUID 查找任务记录、导出 JSON。规划器或操作界面可以查看实际完成的技能、失败位置、资源占用和示例世界状态，再决定后续处理。查询不会创建技能、读取新的传感器测量、解除故障或下发动作。

## 查询入口

启动 mock 或 Panda 示例后，在已 source ROS 2 Jazzy 与本仓库 overlay 的终端运行：

```bash
ros2 service call /get_runtime_state robot_interfaces/srv/GetRuntimeState '{}'
ros2 service call /get_execution robot_interfaces/srv/GetExecution "{execution_id: ''}"
python3 scripts/inspect_runtime.py
python3 scripts/inspect_runtime.py --output execution-snapshot.json
```

`execution_id` 为 Action goal UUID 的 16 个字节按顺序转为 **32 位小写十六进制、不含连字符**。Python Action 客户端可使用 `bytes(handle.goal_id.uuid).hex()`。使用客户端已知的 UUID 精确查询：

```bash
python3 scripts/inspect_runtime.py --execution-id <32位十六进制UUID> --output task-record.json
```

`--output` 只创建新文件，不覆盖已有文件。运行时执行线程只维护内存记录，文件写入发生在查询客户端。两个服务调用不构成原子快照；JSON 保留两个响应及各自的 `runtime_id`，客户端发现两次查询之间重启时直接失败。

| 接口/字段 | 含义 |
|---|---|
| `GetRuntimeState` | 当前 busy、活动 UUID、保留的完成 UUID、资源和世界状态 |
| `runtime_id` | 每次进程启动产生的新标识；结合 execution_id 使用 |
| `GetExecution.execution_id` 为空 | 优先活动任务；没有活动任务时返回最近完成任务 |
| UUID 非空 | 只查询该任务，不回退到其他任务 |
| `lookup_status` | active / completed / not_found / no_records |
| `record.steps` | 实际启动过的技能；包含实现、参数、请求 ID 和状态变化 |
| `total_steps` / `completed_steps` | 预期步骤数 / 实际 succeeded 步骤数；失败后未启动的步骤没有执行记录 |
| `recent_execution_ids` | 完成记录从新到旧，活动任务单独显示 |
| `evicted_records` | 本进程因容量上限淘汰的完成记录数量 |

任务开始后立即建立记录，因此在第一个技能启动前超时也有记录。接收前被拒绝的请求没有执行记录。`ExecuteTask` 和 `ExecutePlan` 使用同一个记录与执行生命周期，原有 Action 字段不变。

## 状态证据

`WorldSnapshot` 明确分开 observations、attached_object、known_locations、placement_candidates 和 resource_leases。当前两个后端都使用 `ConfiguredDemoLocator`，所以观测来源为 `configured_demo`。其位姿为示例运动目标，不能当作真实物体的视觉姿态。新增真实感知组件时必须同时扩展来源、质量和坐标变换契约，不能保留这个合成来源标记。

- 位姿数组顺序为 x,y,z,qx,qy,qz,qw；位置单位米，包含 frame_id。
- `age_ms` 相对于快照采集的单调时钟，只有 `stamp_valid=true` 时有意义；它不代表观测通过了某个技能的时效要求，也不是 ROS 时间戳。
- 任务记录中的世界与资源快照冻结在该任务完成时；当前状态服务重新计算观测年龄。历史年龄不会随之后的查询增长。
- attached_object/known_locations 由示例夹持反馈更新；释放只生成 placement_candidates。任务 succeeded 不能证明物体实际在托盘上。
- `stop_confirmed` 表示本运行时会话和资源已经静止/释放，依据现有组件反馈；它不覆盖外部控制源，也不是独立硬件停机保证。活动任务尚未下发动作时也可能为 true。
- busy 表示当前占有 Action 执行入口；故障任务结束后可以 busy=false，但 fault_latched=true、stop_confirmed=false 和资源占用仍阻止下一任务。

取消接受时记录 canceling，保留资源并等待反馈；停止确认后才记录 canceled/timed_out。超过停止期限记录 faulted/STOP_UNCONFIRMED。此时某个技能仍可能为 canceling，任务终态不会伪造该技能的停止确认。若后续反馈确认停止，当前资源状态可以变化，但完成时的历史证据保持不变，故障锁定仍需独立恢复流程。

技能记录只在状态变化时追加 transition，持续 running 的进度消息更新当前结果，不为每个 timer tick 追加一条。最多 32 个技能、每技能 8 次状态变化；当前顺序技能通常仅有 1～3 次。

## 保留与时间

启动只读参数 `execution_history_capacity` 默认为 32，可设 1～128。最多保留此数量的完成任务和一个活动任务，达到上限淘汰最旧完成任务。进程重启清空内存并改变 runtime_id；JSON 导出是独立诊断文件。

开始时间 `started_unix_ms` 用系统时钟便于日志对齐；任务持续时间及技能 transition 的 elapsed_ms 用单调时钟，不受系统时间调整影响。历史记录查询返回副本，不会改写时间、事实或资源。

`not_found` 可能表示淘汰、进程重启、请求被拒绝或查错实例，**不能证明任务没有执行**。这些记录不是持久幂等账本，没有重连恢复、动作回放、自动重试、完整传感器序列或高频训练数据。客户端查询失败后不能重新提交机器人动作来“恢复查询”。

## 验证

核心测试覆盖有界保留、重复开始、记录冻结、查询副本、取消确认和异常资源保留。ROS 测试覆盖空状态/拒绝请求、按 UUID 查询、两种 Action、运行与取消中的资源归属、超时/失败、STOP_UNCONFIRMED 锁定、容量与只读参数、JSON 导出和文件防覆盖；Panda 测试查询真实 MoveIt/控制器链路产生的任务记录。

mock 增加启动参数 `mock_stop_ticks`（默认 2，范围 1～100000），用于模拟延迟停止，验证未确认停止时的查询结果。只作用于纯 mock 手臂/夹爪；不修改 Panda ROS 适配器的停止反馈规则。
