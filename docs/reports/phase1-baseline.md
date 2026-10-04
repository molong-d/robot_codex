# 阶段一：环境与现有功能基线报告

## 测试对象

- 分支：`sim/gazebo-panda-pick-place`
- 被测试代码提交：`de52ed51330045e75de40fcc1f118bb9467fd754`
- 上游测试基线：`stable/v0.4.0-targets-20261004`，`40353703cc13eb1c53434adc62cd749e283af1e6`
- 测试时间：2026-10-04 03:44:47–03:47:01 UTC
- 测试开始前跟踪文件工作区干净；清除了此前生成的 `build/`、`install/`、`log/` 后从头构建。测试输出只写入 Git 忽略目录。
- 完整终端记录：[phase1-baseline.log](phase1-baseline.log)（358 KiB，SHA-256 `c224d263ac68641d17a60230b67cf88fc993be119395bcacea20dc6d217eb1e6`）；实际 ROS 包版本：[ros-jazzy-packages.txt](ros-jazzy-packages.txt)（288 个包）。

## 实测环境

宿主机是 Ubuntu 22.04.5 LTS、内核 `6.8.0-138-generic`，Intel Core i5-14600KF（20 个逻辑 CPU）、31 GiB 内存。已有 ROS Noetic 与 ROS 2 Humble 安装；本次没有修改宿主 ROS 或 apt 软件包。测试在 Docker 的 Ubuntu 24.04.5 / ROS 2 Jazzy 容器中运行，Docker Engine 为 29.3.1。NVIDIA 设备未能由 `nvidia-smi` 连接，显示环境为 `:1`；未使用 GUI 或 GPU。

容器基础镜像 `ros:jazzy-ros-base` 固定为 OCI digest `sha256:066420e07f60aa18262f2479981def87ebcfcec42eefb0c0c57c4a46098348ca`，本地构建镜像 ID 为 `sha256:e78120951b67089bbad339217778e865734801e5faa4d360c13e70e1dd40d7d1`。测试时的关键版本：

| 组件 | 实际版本 |
|---|---|
| Ubuntu 容器 | 24.04.5 LTS (Noble) |
| ROS 2 | Jazzy |
| Python / GCC / CMake | 3.12.3 / 13.3.0 / 3.28.3 |
| BehaviorTree.CPP | 4.10.0 |
| MoveIt 2 (`moveit_core`) | 2.12.4 |
| ros2_control (`controller_manager`, `hardware_interface`) | 4.48.0 |
| 轨迹与夹爪控制器 | 4.42.1 |
| Gazebo Sim / Harmonic | 8.15.0 |
| `gz_ros2_control` | 1.2.20 |
| `ros_gz` bridge / sim | 1.0.24 |
| colcon-core | 0.21.3 |

APT 来自文档所列 USTC Ubuntu 与 ROS 镜像；仓库依赖由 `rosdep` 解析。精确 ROS deb 版本见包清单。基础镜像 digest 固定，但 apt 仓库会更新，因此清单记录的是本次实际解析到的版本，不声称 apt 仓库为不可变快照。

## 实际执行结果

统一复现入口：

```bash
scripts/with_jazzy.sh scripts/test_all_jazzy.sh
```

该命令在提交 `de52ed5`、干净工作区上退出码为 **0**；脚本遇到任一失败即停止。脚本依次运行下列命令，全部通过：

| 命令 | 结果 | 退出码 |
|---|---|---:|
| `bash scripts/test_core.sh` | 59 项通过 | 0 |
| `colcon build --event-handlers console_direct+ --cmake-args -DBUILD_TESTING=ON` | 6 个包构建完成，27.1 秒 | 0 |
| `colcon test --event-handlers console_direct+ --return-code-on-test-failure` | 6 个包测试完成 | 0 |
| `colcon test-result --verbose` | 1 项 ament 测试，0 错误、0 失败、0 跳过 | 0 |
| `python3 scripts/test_ros.py` | 37 项通过，40.696 秒 | 0 |
| `python3 scripts/test_panda.py` | 11 项通过，61.585 秒 | 0 |

环境入口也已从干净基础镜像完整构建：

```bash
docker build --network host --file docker/jazzy.Dockerfile \
  --build-arg UBUNTU_MIRROR=https://mirrors.ustc.edu.cn/ubuntu \
  --build-arg ROS_MIRROR=https://mirrors.ustc.edu.cn/ros2/ubuntu \
  --tag robot-codex-jazzy:local .
```

最终镜像构建退出码为 0；随后用 `scripts/with_jazzy.sh` 验证了 ROS 发行版、MoveIt、Gazebo 与 `gz_ros2_control` 的安装可见性，再运行上表完整测试。`scripts/test_all_jazzy.sh` 与 `.github/workflows/ci.yml` 使用相同的构建、colcon 和 Python 测试命令。

## 覆盖范围与限制

- 核心测试覆盖合法与非法输入、成功、动作/组件失败、超时、取消、停止确认、忙碌资源占用、租约释放、异常后保留未知占用和位姿新鲜度/证据验证。ROS 测试另覆盖计划拒绝、反馈验证和执行记录。Panda 测试验证 MoveIt、控制器、超时取消和子进程关闭路径。
- Panda 当前仍使用 `GenericSystem` / fake gripper 与合成抓取/放置证据。日志明确打印 `no physical object/contact sensing`。本阶段没有运行 Gazebo 世界，没有测试真实接触、物体附着、视觉、物理放置或仿真时钟；通过结果不能解释为物理仿真或实机验证。
- 两次 Panda 测试的 SIGINT 关闭期间，`controller_manager.pal_statistics` 发布线程各打印两条 `context cannot be slept with because it's invalid` 错误；随后 controller manager、MoveIt 和 runtime 子进程均报告 clean exit，测试返回 `OK`。类似 ROS 统计线程的关闭问题已有上游记录：[ros2/ros2 #1659](https://github.com/ros2/ros2/issues/1659)。这里记录为未解决的退出日志告警，不据此断言其根因已确认，也不掩盖测试通过状态。
- 测试日志还包含 `ROS_LOCALHOST_ONLY` 弃用提示；它是仓库 CI 当前使用的环境变量，Jazzy 仍接受该设置。

## 环境搭建中遇到的问题

- 宿主 ROS 已有 Humble/Noetic，故采用隔离容器；未升级或覆盖宿主安装。一次早期 bootstrap 用 `set -u` 后在 source ROS setup 时遇到未设置的 `AMENT_TRACE_SETUP_FILES`，测试尚未开始；改为 `set -e` 后重跑。
- USTC ROS 镜像未提供 `deb-src` 索引，安装容器中仅保留 `Types: deb`。Docker BuildKit 默认网络访问 rosdistro 的 GitHub raw 文件被重置；构建入口改用 `--network host` 后 `rosdep update` 成功。
- 构建配方验证还发现 Buildx 默认状态目录不可写、rosdep 运行前过早删除 APT 索引及一次 Dockerfile 续行错误。入口将 Buildx 配置置于临时目录，APT 索引延至 rosdep 安装后清理，并修复续行。上述是搭建过程失败，不计入测试通过数；最终镜像从基础镜像构建成功并通过完整复测。

## 下一阶段边界

阶段一已复现项目当前 mock、ROS 和 Panda GenericSystem 路径。阶段二需要另建 Gazebo Harmonic 入口，并以 Gazebo 世界物体状态作为独立结果证据；本报告不声称那些能力已存在或已验证。
