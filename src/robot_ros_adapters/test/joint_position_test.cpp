#include "robot_ros_adapters/joint_position.hpp"
#include <limits>
#include <stdexcept>

void require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}

int main() {
  double position = -1.0;
  require(robot_ros_adapters::panda_joint_position(-2.8e-18, position),
          "DART zero-limit roundoff was rejected");
  require(position == 0.0, "negative roundoff was not clamped to zero");
  require(robot_ros_adapters::panda_joint_position(0.04, position) && position == 0.04,
          "valid finger position was changed");
  require(!robot_ros_adapters::panda_joint_position(-1e-12, position),
          "negative position outside the roundoff bound was accepted");
  require(!robot_ros_adapters::panda_joint_position(std::numeric_limits<double>::quiet_NaN(), position),
          "non-finite joint position was accepted");
}
