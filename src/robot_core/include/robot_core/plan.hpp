#pragma once
#include "robot_core/task_catalog.hpp"

namespace robot_core {
struct PlanStep {
  std::string skill;
  std::string implementation;
  Arguments arguments;
};
struct Plan {
  unsigned schema_version{1};
  std::vector<PlanStep> steps;
};

// Bounded sequential plans for the five current skill semantics. This does
// not interpret descriptive conditions, predict sensor data, or alter world state.
inline void validate_plan(const Plan& plan, const DemoScene& scene, const Skills& skills,
                          const Bindings& bindings, const WorldState& world) {
  if (plan.schema_version != 1 || plan.steps.empty() || plan.steps.size() > 32)
    throw std::invalid_argument("plan requires schema version 1 and 1..32 steps");
  std::set<std::string> located;
  auto held = world.attached_object;
  auto released = world.placement_candidates;
  if (!held.empty() && scene.at(held).role != EntityRole::object)
    throw std::invalid_argument("held object is not in configured object catalog");
  for (size_t i = 0; i < plan.steps.size(); ++i) {
    try {
      const auto& step = plan.steps[i];
      if (!valid_id(step.skill) || !valid_id(step.implementation))
        throw std::invalid_argument("invalid skill or implementation ID");
      if (step.skill != "locate_object" && step.skill != "pick_object" && step.skill != "place_object" &&
          step.skill != "verify_grasp" && step.skill != "verify_placement")
        throw std::invalid_argument("unsupported plan skill semantics");
      skills.validate_arguments(step.skill, step.arguments);
      skills.validate_dependencies(step.skill, step.implementation, bindings);
      const auto& object = step.arguments.at("object");
      const auto role = scene.at(object).role;
      if (step.skill == "locate_object") {
        located.insert(object);
      } else if (step.skill == "pick_object") {
        if (role != EntityRole::object || !held.empty() || located.erase(object) == 0)
          throw std::invalid_argument("pick requires an object, preceding locate and empty gripper");
        held = object;
        released.erase(object);
      } else if (step.skill == "verify_grasp") {
        if (role != EntityRole::object || held != object)
          throw std::invalid_argument("grasp verification requires matching held object");
      } else if (step.skill == "place_object") {
        const auto& target = step.arguments.at("target");
        if (role != EntityRole::object || scene.at(target).role != EntityRole::target ||
            held != object || located.erase(target) == 0)
          throw std::invalid_argument("place requires matching held object and preceding target locate");
        held.clear();
        released[object] = target;
      } else {
        const auto& target = step.arguments.at("target");
        const auto it = released.find(object);
        if (role != EntityRole::object || scene.at(target).role != EntityRole::target || held == object ||
            it == released.end() || it->second != target)
          throw std::invalid_argument("placement verification requires matching preceding release candidate");
        released.erase(it);
      }
    } catch (const std::exception& e) {
      throw std::invalid_argument("plan step " + std::to_string(i) + ": " + e.what());
    }
  }
}
}  // namespace robot_core
