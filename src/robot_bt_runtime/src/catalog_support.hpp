#pragma once
#include "robot_core/task_catalog.hpp"
#include "robot_interfaces/srv/get_catalog.hpp"
#include <rclcpp/rclcpp.hpp>

namespace catalog_support {
namespace rc = robot_core;
using GetCatalog = robot_interfaces::srv::GetCatalog;

template<class T> T startup_parameter(rclcpp::Node* node, const std::string& name, const T& value) {
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.read_only = true;
  descriptor.description = "Startup configuration; restart the demo to change it";
  return node->declare_parameter<T>(name, value, descriptor);
}

inline rc::DemoScene load_scene(rclcpp::Node* node, const std::string& frame, bool panda) {
  std::vector<rc::DemoEntity> entities;
  for (const auto role : {rc::EntityRole::object, rc::EntityRole::target}) {
    const auto ids = startup_parameter<std::vector<std::string>>(node,
        role == rc::EntityRole::object ? "object_ids" : "target_ids",
        role == rc::EntityRole::object ? std::vector<std::string>{"workpiece"} : std::vector<std::string>{"tray"});
    for (const auto& id : ids) {
      if (!rc::valid_id(id)) throw std::invalid_argument("invalid configured entity ID: " + id);
      std::vector<double> fallback;
      // Preserve the v0.3 CLI demo when no parameter file is supplied.
      if (id == "workpiece") fallback = panda ? std::vector<double>{0.4, 0.1, 0.4, 1.0, 0.0, 0.0, 0.0} :
                                                std::vector<double>{0.4, 0.1, 0.2, 0.0, 0.0, 0.0, 1.0};
      if (id == "tray") fallback = panda ? std::vector<double>{0.45, -0.15, 0.4, 1.0, 0.0, 0.0, 0.0} :
                                            std::vector<double>{0.6, -0.2, 0.15, 0.0, 0.0, 0.0, 1.0};
      const auto pose = startup_parameter<std::vector<double>>(node, "entities." + id + ".pose", fallback);
      if (pose.size() != 7) throw std::invalid_argument("entity pose needs x,y,z,qx,qy,qz,qw: " + id);
      entities.push_back({id, role, {pose[0], pose[1], pose[2], pose[3], pose[4], pose[5], pose[6]}});
    }
  }
  return rc::DemoScene(frame, std::move(entities));
}

inline rc::TaskCatalog load_tasks(rclcpp::Node* node) {
  const auto names = startup_parameter<std::vector<std::string>>(node, "task_names", {"pick_place", "inspect_object"});
  std::vector<rc::TaskDefinition> tasks;
  for (const auto& name : names) {
    if (!rc::valid_id(name)) throw std::invalid_argument("invalid configured task name: " + name);
    const std::string fallback = name == "pick_place" ? "pick_place" :
                                 name == "verified_pick_place" ? "verified_pick_place" :
                                 name == "inspect_object" ? "locate_object" : "";
    tasks.push_back({name, startup_parameter<std::string>(node, "tasks." + name + ".template_id", fallback),
                    startup_parameter<std::string>(node, "tasks." + name + ".implementation_id", "standard")});
  }
  return rc::TaskCatalog(std::move(tasks));
}

inline void describe(const rc::Skills& skills, const rc::Bindings& bindings,
                     const rc::TaskCatalog& tasks, const rc::DemoScene& scene, GetCatalog::Response& response) {
  response.schema_version = 1;
  response.object_ids = scene.ids(rc::EntityRole::object);
  response.target_ids = scene.ids(rc::EntityRole::target);
  for (const auto& skill : skills.catalog(bindings)) {
    robot_interfaces::msg::SkillInfo info;
    info.skill_id = skill.definition.id;
    info.description = skill.definition.description;
    info.precondition = skill.definition.precondition;
    info.invariant = skill.definition.invariant;
    info.success_condition = skill.definition.success_condition;
    for (const auto& input : skill.definition.inputs) {
      robot_interfaces::msg::SkillInput parameter;
      parameter.name = input.name; parameter.type = input.type;
      parameter.required = true; parameter.description = input.description;
      info.inputs.push_back(std::move(parameter));
    }
    for (const auto& implementation : skill.implementations) {
      robot_interfaces::msg::SkillImplementation impl;
      impl.implementation_id = implementation.id;
      impl.exclusive_roles = implementation.dependencies.exclusive_roles;
      impl.execution_gate_role = implementation.dependencies.execution_gate_role;
      impl.dependencies_satisfied = implementation.dependencies_satisfied;
      impl.unavailable_reason = implementation.unavailable_reason;
      for (const auto& requirement : implementation.dependencies.components) {
        robot_interfaces::msg::ComponentRequirement component;
        component.role = requirement.role; component.interface_id = requirement.interface_id;
        component.interface_version = requirement.version;
        impl.components.push_back(std::move(component));
      }
      info.implementations.push_back(std::move(impl));
    }
    response.skills.push_back(std::move(info));
  }
  for (const auto& entry : tasks.definitions()) {
    robot_interfaces::msg::TaskInfo info;
    info.task_name = entry.second.name; info.template_id = entry.second.template_id;
    info.implementation_id = entry.second.implementation;
    info.required_inputs = rc::TaskCatalog::required_inputs(entry.second);
    response.tasks.push_back(std::move(info));
  }
}
}  // namespace catalog_support
