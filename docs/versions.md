# 版本与回退

## 稳定基线

| 备份分支 | 提交 | 内容 |
|---|---|---|
| `stable/v0.2.0-20261003` | `7ca0a4c21a4ca2f62fbd1081dedc32642d873b69` | PR #3 合并后的 mock 基线；Jazzy CI 通过 |
| `stable/v0.3.0-20261003` | `3ebcf9be59fd53f82abf80a835a35f7af785d946` | PR #4/#5 合并后基线；代码树与通过 Jazzy CI 的 Panda 示例一致 |
| `stable/v0.3.0-contracts-20261003` | `9798623861b6b13c0ac78fcb38ba2002d900a58b` | 4A 契约阶段通过测试的快照 |
| `stable/v0.3.0-panda-demo-20261003` | `5c879f08a389876bb06b3790c8b1c022003001a0` | 4B Panda 阶段通过测试的快照 |
| `stable/v0.4.0-catalog-r1-20261003` | `d2ec8e8768c21ccb1c836077c899b50d33ea585c` | 5A 修订快照；功能测试与 Panda 子进程退出检查通过 |
| `stable/v0.4.0-plans-20261003` | `ba8368dbedc7216e0cdd47f88192cc5be4d08085` | 5B 快照；核心、ROS 结构化计划及 Panda 测试通过 |

备份分支固定指向原提交，后续开发不向该分支推送。备份保留的是当时经过测试的软件行为，不代表已经通过实机或功能安全验证。

初版 `stable/v0.4.0-catalog-20261003`（`2fcfe9c`）保留作追溯，但发现 MoveIt 退出崩溃，复现 5A 请使用 `catalog-r1` 修订快照。5B 使用独立分支和堆叠 PR；先合并 5A，再调整 5B 的目标为 main，核对差异并合并。

6A 分支 `framework/execution-records-v04` 在 5B 基础上开发；合并顺序为 5A → 5B → 6A。每次前置 PR 合并后，调整下一 PR 的目标为 main，再检查差异和 CI。6A 验证通过后创建新的 `stable/v0.4.0-records-20261003` 快照，保留全部旧备份；主线合并前继续以 v0.3 作为已合并稳定基线。

## 升级流程

每个阶段使用独立开发分支和 PR；通过核心和 ROS CI 后由所有者确认合并。合并后的新基线再创建新的 `stable/` 备份分支，保留旧备份。避免用强制推送覆盖主线或备份。

4A 将 `arm_motion` 与 `gripper` 契约升级为版本 2：`poll(Time)` 与带时间戳的测量反馈替代旧的布尔到位/夹持查询。旧实现应编译失败或在绑定时被拒绝，不能伪装成兼容实现。

5A 开发版使用 `SkillDefinition::inputs` 取代 `required_inputs`，增加输入类型与描述。迁移细节见 [技能目录与任务配置](task_catalog.md)。`ExecuteTask` 的 ROS 字段和 v0.3 默认请求不变；新实体和任务别名在启动配置中增加。主分支合并前继续以 v0.3 备份作为稳定基线。

## 在独立目录复现旧版本

```bash
git clone --branch stable/v0.2.0-20261003 https://github.com/molong-d/robot_codex.git robot_codex-v02
cd robot_codex-v02
bash scripts/test_core.sh
```

在独立目录构建旧版本，分别使用各自的 build/install/log；不要混用 ROS overlay。若需要回退主线，优先使用经审阅的 revert 提交，避免改写共享提交历史。
