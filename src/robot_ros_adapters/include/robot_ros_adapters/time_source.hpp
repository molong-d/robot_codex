#pragma once
#include "robot_core/components.hpp"

namespace robot_ros_adapters {

// Maps source ROS/simulation time to steady time only when the source stamp
// advances. Repeated messages while Gazebo is paused retain their old stamp and
// ID; a backward jump starts a new epoch without reusing sample IDs.
class MonotonicSampleClock {
 public:
  bool observe(int64_t source_ns, int64_t clock_ns, robot_core::Time receipt) {
    constexpr int64_t max_age_ns = 500000000;
    if (source_ns <= 0 || clock_ns < source_ns || clock_ns-source_ns > max_age_ns) return false;
    if (source_ns == source_ns_) return false;
    if (source_ns_ > 0 && source_ns < source_ns_) ++epoch_;
    const auto age = std::chrono::nanoseconds(clock_ns-source_ns);
    auto mapped = receipt-std::chrono::duration_cast<robot_core::Clock::duration>(age);
    if (sample_id_ != 0 && mapped <= stamp_) mapped = stamp_+robot_core::Clock::duration{1};
    source_ns_ = source_ns;
    stamp_ = mapped;
    ++sample_id_;
    return true;
  }
  robot_core::Time stamp() const { return stamp_; }
  uint64_t sample_id() const { return sample_id_; }
  uint64_t epoch() const { return epoch_; }
 private:
  int64_t source_ns_{0};
  robot_core::Time stamp_{};
  uint64_t sample_id_{0}, epoch_{0};
};
}  // namespace robot_ros_adapters
