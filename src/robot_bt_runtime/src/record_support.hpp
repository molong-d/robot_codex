#pragma once
#include "robot_core/execution_record.hpp"
#include "robot_interfaces/srv/get_execution.hpp"
#include "robot_interfaces/srv/get_runtime_state.hpp"
#include <array>
#include <random>

namespace record_support {
namespace rc = robot_core;
namespace msg = robot_interfaces::msg;
using GetExecution = robot_interfaces::srv::GetExecution;
using GetRuntimeState = robot_interfaces::srv::GetRuntimeState;

template<class Bytes> std::string hex_id(const Bytes& bytes) {
  constexpr char digits[] = "0123456789abcdef";
  std::string value;
  value.reserve(bytes.size()*2);
  for (const auto byte : bytes) {
    value += digits[static_cast<unsigned>(byte) >> 4];
    value += digits[static_cast<unsigned>(byte) & 15];
  }
  return value;
}
inline std::string runtime_id() {
  std::random_device random;
  std::array<uint8_t, 16> bytes{};
  for (auto& byte : bytes) byte = static_cast<uint8_t>(random());
  return hex_id(bytes);
}
inline uint64_t unix_ms() {
  return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count());
}
inline msg::WorldSnapshot describe(const rc::ExecutionSnapshot& snapshot) {
  msg::WorldSnapshot result;
  result.attached_object = snapshot.world.attached_object;
  result.resources_empty = snapshot.resources_empty;
  result.fault_latched = snapshot.fault_latched;
  result.stop_confirmed = snapshot.stop_confirmed;
  for (const auto& entry : snapshot.world.observations) {
    const auto& o = entry.second;
    msg::ObservationSnapshot observation;
    observation.entity_id = o.object_id; observation.frame_id = o.frame_id;
    observation.pose = {o.pose.x, o.pose.y, o.pose.z, o.pose.qx, o.pose.qy, o.pose.qz, o.pose.qw};
    // Both current backends use ConfiguredDemoLocator, not vision.
    observation.source = "configured_demo";
    observation.valid = o.valid;
    observation.stamp_valid = o.stamp != rc::Time{} && o.stamp <= snapshot.captured_at;
    if (observation.stamp_valid)
      observation.age_ms = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
          snapshot.captured_at-o.stamp).count());
    result.observations.push_back(std::move(observation));
  }
  const auto locations = [](const auto& entries, auto& output) {
    for (const auto& entry : entries) {
      msg::EntityLocation location;
      location.entity_id = entry.first; location.location_id = entry.second;
      output.push_back(std::move(location));
    }
  };
  locations(snapshot.world.known_locations, result.known_locations);
  locations(snapshot.world.placement_candidates, result.placement_candidates);
  for (const auto& entry : snapshot.resource_owners) {
    msg::ResourceLease lease;
    lease.resource_id = entry.first; lease.owner_request_id = entry.second;
    result.resource_leases.push_back(std::move(lease));
  }
  return result;
}
inline msg::ExecutionRecord describe(const rc::ExecutionRecord& record) {
  msg::ExecutionRecord result;
  result.execution_id = record.id; result.entrypoint = record.entrypoint;
  result.task_name = record.task_name; result.started_unix_ms = record.started_unix_ms;
  result.elapsed_ms = record.elapsed_ms; result.timeout_ms = record.timeout_ms;
  result.total_steps = record.total_steps; result.status = rc::name(record.result.status);
  result.error_code = record.result.code; result.message = record.result.message;
  result.snapshot = describe(record.snapshot);
  for (const auto& entry : record.steps) {
    msg::SkillExecutionRecord step;
    step.step_index = static_cast<uint32_t>(result.steps.size());
    step.request_id = entry.request.id; step.step.skill_id = entry.request.skill;
    step.step.implementation_id = entry.request.implementation;
    for (const auto& argument : entry.request.arguments) {
      step.step.argument_names.push_back(argument.first); step.step.argument_values.push_back(argument.second);
    }
    step.started_ms = entry.started_ms; step.updated_ms = entry.updated_ms;
    step.status = rc::name(entry.result.status); step.error_code = entry.result.code; step.message = entry.result.message;
    if (entry.result.status == rc::Status::succeeded) ++result.completed_steps;
    for (const auto& value : entry.transitions) {
      msg::StateTransition transition;
      transition.elapsed_ms = value.elapsed_ms; transition.status = rc::name(value.result.status);
      transition.error_code = value.result.code; transition.message = value.result.message;
      step.transitions.push_back(std::move(transition));
    }
    result.steps.push_back(std::move(step));
  }
  return result;
}
}  // namespace record_support
