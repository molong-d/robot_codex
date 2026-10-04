# Ubuntu / ROS 2 开发环境

## 推荐方式

仓库兼容目标是 Ubuntu 24.04、ROS 2 Jazzy。若宿主机已有其他 ROS 发行版，使用仓库提供的容器脚本隔离依赖；脚本不会安装或改写宿主机 ROS。需要 Docker Engine，镜像第一次构建会从 Ubuntu 与 ROS 软件源获取依赖。

```bash
bash scripts/with_jazzy.sh
bash scripts/with_jazzy.sh bash scripts/test_all_jazzy.sh
```

也可以在容器 shell 中逐项执行，命令与 CI 一致：

```bash
bash scripts/test_core.sh
colcon build --event-handlers console_direct+ --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
colcon test --event-handlers console_direct+ --return-code-on-test-failure
colcon test-result --verbose
python3 scripts/test_ros.py
python3 scripts/test_panda.py
```

容器运行时将当前仓库挂载到 `/workspace`，构建产物留在本地仓库的忽略目录。当前默认使用 USTC Ubuntu 与 ROS 镜像以提升此开发环境的下载速度。网络策略不允许使用该镜像时可改用官方源：

```bash
ROBOT_CODEX_UBUNTU_MIRROR=http://archive.ubuntu.com/ubuntu \
ROBOT_CODEX_ROS_MIRROR=http://packages.ros.org/ros2/ubuntu \
bash scripts/with_jazzy.sh
```

`ROBOT_CODEX_JAZZY_IMAGE` 可指定本地镜像名。基础镜像固定为 `ros:jazzy-ros-base` 的 OCI digest `sha256:066420e07f60aa18262f2479981def87ebcfcec42eefb0c0c57c4a46098348ca`；APT 依赖按构建时软件源版本安装，具体安装包版本见对应测试报告和 [ROS Jazzy 包版本清单](reports/ros-jazzy-packages.txt)。仓库源码依赖由 `rosdep` 按 Jazzy 规则解析。不要把宿主机 `build/`、`install/` 或其他 ROS overlay 挂入容器。

## 本仓库一次实测的环境

阶段一基线在 Ubuntu 24.04.5 容器中通过上述依赖和脚本复现，容器运行于 Ubuntu 22.04.5 宿主机。测试时 ROS 2 Jazzy、MoveIt 2 2.12.4、BehaviorTree.CPP 4.10.0、ros2_control 4.48.0、Gazebo Harmonic 8.15.0、`gz_ros2_control` 1.2.20。完整包版本清单与测试命令、提交 SHA、退出码见 [`phase1-baseline.md`](reports/phase1-baseline.md)。宿主机无可用 NVIDIA 驱动；后续仿真应以无 GUI/headless 模式验证，不以该机器的渲染能力作假设。

此容器脚本用于开发与复现，不会自动启动 Gazebo、连接真实机器人或下载大型模型权重。第一阶段 Panda 示例使用 `GenericSystem`；它验证 ROS/MoveIt 接口路径，不验证接触物理。
