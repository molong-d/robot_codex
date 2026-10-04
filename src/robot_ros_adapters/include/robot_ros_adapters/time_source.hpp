#pragma once
#include "robot_core/components.hpp"

namespace robot_ros_adapters {

// Maps a source ROS/simulation stamp to steady receipt time. Source ordering is
// checked independently from the ROS clock: an old sensor message received
// while /clock advances is rejected, while a confirmed /clock rewind starts a
// new epoch. Repeated messages while paused never refresh evidence.
class MonotonicSampleClock {
 public:
  explicit MonotonicSampleClock(std::chrono::milliseconds max_age = std::chrono::milliseconds(500))
      : max_age_ns_(std::chrono::duration_cast<std::chrono::nanoseconds>(max_age).count()) {
    if (max_age.count() <= 0) throw std::invalid_argument("sample max age must be positive");
  }
  bool observe(int64_t source_ns, int64_t clock_ns, robot_core::Time receipt) {
    constexpr int64_t reset_confirmation_ns = 100000000;
    if (clock_ns <= 0) return false;
    if (clock_ns_ns_ > reset_confirmation_ns &&
        clock_ns < clock_ns_ns_ - reset_confirmation_ns) {
      ++epoch_;
      reset_reference_ns_ = source_ns_;
      awaiting_reset_sample_ = reset_reference_ns_ > reset_confirmation_ns;
      source_ns_ = 0;
      stamp_ = {};
    } else if (clock_ns < clock_ns_ns_) {
      // Small clock jitter is not a new epoch and must not admit a sample.
      return false;
    }
    clock_ns_ns_ = clock_ns;
    if (source_ns <= 0 || clock_ns < source_ns || clock_ns-source_ns > max_age_ns_) return false;
    if (awaiting_reset_sample_) {
      // Keep the stream invalid until ROS time passes the previous source
      // high-water mark; this prevents a delayed in-flight pre-reset callback
      // from becoming the first valid sample in the new epoch.
      if (source_ns <= reset_reference_ns_) return false;
      awaiting_reset_sample_ = false;
    }
    if (source_ns <= source_ns_) return false;
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
  int64_t source_ns() const { return source_ns_; }
 private:
  int64_t source_ns_{0}, clock_ns_ns_{0}, reset_reference_ns_{0};
  int64_t max_age_ns_{500000000};
  robot_core::Time stamp_{};
  uint64_t sample_id_{0}, epoch_{0};
  bool awaiting_reset_sample_{false};
};
}  // namespace robot_ros_adapters
