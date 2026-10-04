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

  // A clock reset starts an epoch while keeping the monotonic identity and
  // mapped timestamp increasing; old and new evidence cannot be conflated.
  require(source.observe(100'000'000, 100'000'000, steady0+3s), "reset epoch sample was rejected");
  require(source.sample_id() == 3 && source.epoch() == 1 && source.stamp() == steady0+3s,
          "clock reset reused sample identity or failed to advance epoch");
  require(source.observe(200'000'000, 200'000'000, steady0+3100ms), "post-reset sample was rejected");
  require(source.sample_id() == 4 && source.epoch() == 1 && source.stamp() > steady0+3s,
          "post-reset sample time did not remain monotonic");

  // Future and stale source messages are rejected without changing the cache.
  require(!source.observe(500'000'000, 200'000'000, steady0+4s), "future sample was accepted");
  require(!source.observe(100'000'000, 1'000'000'000, steady0+4s), "stale sample was accepted");
  require(source.sample_id() == 4 && source.epoch() == 1, "rejected sample changed clock state");
}
