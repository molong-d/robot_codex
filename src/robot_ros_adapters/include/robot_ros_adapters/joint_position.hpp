#pragma once
#include <algorithm>
#include <cmath>
#include <limits>

namespace robot_ros_adapters {

inline bool panda_joint_position(double raw, double& position) {
  // DART reports values around -3e-18 rad for Panda fingers resting at their
  // zero limit. Clamp only floating-point roundoff; reject real negative values.
  constexpr double roundoff = 16.0 * std::numeric_limits<double>::epsilon();
  if (!std::isfinite(raw) || raw < -roundoff) return false;
  position = std::max(0.0, raw);
  return true;
}

}  // namespace robot_ros_adapters
