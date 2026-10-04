#include "robot_ros_adapters/time_source.hpp"
#include <stdexcept>

using namespace std::chrono_literals;
using robot_core::Clock;
using robot_ros_adapters::MonotonicSampleClock;

void require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}

int main() {
  MonotonicSampleClock source;
  const auto steady0 = Clock::time_point{10s};
  require(source.observe(1'000'000'000, 1'000'000'000, steady0), "first advancing sample was rejected");
  require(source.sample_id() == 1 && source.epoch() == 0 && source.stamp() == steady0,
          "first sample identity or mapped time is incorrect");

  // A paused simulator may deliver a cached message repeatedly. Its stamp and
  // ID stay fixed, so polling cannot turn it into new evidence.
  require(!source.observe(1'000'000'000, 1'000'000'000, steady0+2s), "duplicate paused sample was accepted");
  require(source.sample_id() == 1 && source.stamp() == steady0, "duplicate sample refreshed evidence");
  require(!robot_core::fresh(source.stamp(), steady0+600ms, 500ms), "paused evidence remained fresh");

  // Resume advances the simulation stamp and produces a new steady sample.
  require(source.observe(2'000'000'000, 2'000'000'000, steady0+2s), "resumed sample was rejected");
  require(source.sample_id() == 2 && source.stamp() == steady0+2s, "resume did not advance sample time");

  // A confirmed /clock reset advances the epoch immediately, but each stream
  // remains invalid until its restarted source passes the old source watermark.
  require(!source.observe(100'000'000, 100'000'000, steady0+3s), "low post-reset source crossed the barrier");
  require(source.sample_id() == 2 && source.epoch() == 1,
          "clock reset reused sample identity or failed to advance epoch");
  require(!source.observe(1'950'000'000, 1'950'000'000, steady0+3100ms),
          "delayed old-epoch source crossed the reset barrier");
  require(source.sample_id() == 2 && source.epoch() == 1,
          "rejected reset-barrier sample refreshed identity");
  require(source.observe(2'100'000'000, 2'100'000'000, steady0+4s),
          "post-reset stream failed to recover after crossing old watermark");
  require(source.sample_id() == 3 && source.epoch() == 1 && source.stamp() > steady0+3s,
          "post-reset sample identity/time did not remain monotonic");

  // A delayed old sensor message while ROS time advances is out of order, not
  // a simulator reset, and cannot refresh identity or freshness.
  MonotonicSampleClock ordered;
  require(ordered.observe(2'000'000'000, 2'000'000'000, steady0), "ordered first sample rejected");
  require(!ordered.observe(1'900'000'000, 2'100'000'000, steady0+100ms),
          "out-of-order sensor message was accepted while ROS time advanced");
  require(ordered.sample_id() == 1 && ordered.epoch() == 0 && ordered.source_ns() == 2'000'000'000,
          "out-of-order message changed evidence identity");

  // Future and stale source messages are rejected without changing the cache.
  require(!source.observe(2'300'000'000, 2'200'000'000, steady0+4100ms), "future sample was accepted");
  require(!source.observe(1'500'000'000, 2'200'000'000, steady0+4100ms), "stale sample was accepted");
  require(source.sample_id() == 3 && source.epoch() == 1, "rejected sample changed clock state");
}
