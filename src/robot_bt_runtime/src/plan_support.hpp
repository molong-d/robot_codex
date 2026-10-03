#pragma once
#include "robot_core/plan.hpp"
#include "robot_interfaces/action/execute_plan.hpp"
#include <sstream>

namespace plan_support {
using ExecutePlan = robot_interfaces::action::ExecutePlan;
inline robot_core::Plan decode(const ExecutePlan::Goal& goal) {
  if (goal.schema_version != 1 || goal.steps.empty() || goal.steps.size() > 32)
    throw std::invalid_argument("plan requires schema version 1 and 1..32 steps");
  robot_core::Plan plan{goal.schema_version, {}};
  for (const auto& step : goal.steps) {
    if (step.argument_names.size() != step.argument_values.size() || step.argument_names.size() > 2)
      throw std::invalid_argument("plan argument arrays must match and contain at most 2 entries");
    robot_core::Arguments arguments;
    for (size_t i = 0; i < step.argument_names.size(); ++i)
      if (!arguments.emplace(step.argument_names[i], step.argument_values[i]).second)
        throw std::invalid_argument("duplicate plan input name");
    plan.steps.push_back({step.skill_id, step.implementation_id, std::move(arguments)});
  }
  return plan;
}

// Generate trusted node tags from a fully validated plan, never from caller XML.
inline std::string tree_xml(const robot_core::Plan& plan, const robot_core::DemoScene& scene,
                            const robot_core::Skills& skills, const robot_core::Bindings& bindings,
                            const robot_core::WorldState& world) {
  robot_core::validate_plan(plan, scene, skills, bindings, world);
  std::ostringstream xml;
  xml << "<root BTCPP_format=\"4\" main_tree_to_execute=\"SkillPlan\"><BehaviorTree ID=\"SkillPlan\"><Sequence>";
  for (const auto& step : plan.steps) {
    xml << "<Skill skill=\"" << step.skill << "\" implementation=\"" << step.implementation << "\"";
    // All IDs/keys/values have passed the entity_id schema; quotes, braces,
    // dots and XML operators cannot enter the generated ports.
    for (const auto& argument : step.arguments) xml << " " << argument.first << "=\"" << argument.second << "\"";
    xml << "/>";
  }
  xml << "</Sequence></BehaviorTree></root>";
  return xml.str();
}
}  // namespace plan_support
