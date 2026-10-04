FROM ros@sha256:066420e07f60aa18262f2479981def87ebcfcec42eefb0c0c57c4a46098348ca

ARG UBUNTU_MIRROR=https://mirrors.ustc.edu.cn/ubuntu
ARG ROS_MIRROR=https://mirrors.ustc.edu.cn/ros2/ubuntu

ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=jazzy \
    ROS_DOMAIN_ID=42 \
    ROS_LOCALHOST_ONLY=1

RUN sed -i \
      -e "s|http://archive.ubuntu.com/ubuntu|${UBUNTU_MIRROR}|g" \
      -e "s|http://security.ubuntu.com/ubuntu|${UBUNTU_MIRROR}|g" \
      /etc/apt/sources.list.d/ubuntu.sources \
 && sed -i \
      -e 's|^Types:.*|Types: deb|' \
      -e "s|^URIs:.*|URIs: ${ROS_MIRROR}|" \
      /etc/apt/sources.list.d/ros2.sources \
 && apt-get update \
 && apt-get install -y --no-install-recommends \
      python3-colcon-common-extensions \
      python3-rosdep \
      ros-jazzy-behaviortree-cpp \
      ros-jazzy-gz-ros2-control \
      ros-jazzy-ros-gz-bridge \
      ros-jazzy-ros-gz-sim

RUN rosdep update --rosdistro jazzy

COPY src/ /tmp/robot_codex/src/
RUN . /opt/ros/jazzy/setup.sh \
 && rosdep install --from-paths /tmp/robot_codex/src --ignore-src --rosdistro jazzy -r -y \
 && rm -rf /var/lib/apt/lists/* /tmp/robot_codex

COPY docker/jazzy-entrypoint.sh /usr/local/bin/robot-codex-jazzy-entrypoint
RUN chmod 0755 /usr/local/bin/robot-codex-jazzy-entrypoint

WORKDIR /workspace
ENTRYPOINT ["/usr/local/bin/robot-codex-jazzy-entrypoint"]
CMD ["bash"]
