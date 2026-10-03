# 版本与回退

## 稳定基线

| 备份分支 | 提交 | 内容 |
|---|---|---|
| `stable/v0.2.0-20261003` | `7ca0a4c21a4ca2f62fbd1081dedc32642d873b69` | PR #3 合并后的 mock 基线；Jazzy CI 通过 |

备份分支固定指向原提交，后续开发不向该分支推送。备份保留的是当时经过测试的软件行为，不代表已经通过实机或功能安全验证。

## 升级流程

每个阶段使用独立开发分支和 PR；通过核心和 ROS CI 后由所有者确认合并。合并后的新基线再创建新的 `stable/` 备份分支，保留旧备份。避免用强制推送覆盖主线或备份。

4A 将 `arm_motion` 与 `gripper` 契约升级为版本 2：`poll(Time)` 与带时间戳的测量反馈替代旧的布尔到位/夹持查询。旧实现应编译失败或在绑定时被拒绝，不能伪装成兼容实现。

## 在独立目录复现旧版本

```bash
git clone --branch stable/v0.2.0-20261003 https://github.com/molong-d/robot_codex.git robot_codex-v02
cd robot_codex-v02
bash scripts/test_core.sh
```

在独立目录构建旧版本，分别使用各自的 build/install/log；不要混用 ROS overlay。若需要回退主线，优先使用经审阅的 revert 提交，避免改写共享提交历史。
