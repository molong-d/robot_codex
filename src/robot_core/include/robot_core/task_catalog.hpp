#pragma once
#include "robot_core/components.hpp"

namespace robot_core {
enum class EntityRole { object, target };
struct DemoEntity { std::string id; EntityRole role; Pose pose; };

// Fixed, configured poses for demonstrations, NOT a perception/world-state store.
class DemoScene {
 public:
  DemoScene(std::string frame, std::vector<DemoEntity> entities) : frame_(std::move(frame)) {
    if (frame_.empty() || entities.empty()) throw std::invalid_argument("empty demo scene");
    for (auto& entity : entities) {
      if (!valid_id(entity.id) || !valid_pose(entity.pose) ||
          (entity.role != EntityRole::object && entity.role != EntityRole::target))
        throw std::invalid_argument("invalid demo entity: " + entity.id);
      const auto id = entity.id;
      if (!entities_.emplace(id, std::move(entity)).second)
        throw std::invalid_argument("duplicate demo entity: " + id);
    }
  }
  const std::string& frame() const { return frame_; }
  const DemoEntity& at(const std::string& id) const { return entities_.at(id); }
  std::vector<std::string> ids(EntityRole role) const {
    std::vector<std::string> result;
    for (const auto& entity : entities_) if (entity.second.role == role) result.push_back(entity.first);
    return result;
  }
 private:
  std::string frame_;
  std::map<std::string, DemoEntity> entities_;
};

class ConfiguredDemoLocator final : public ObjectLocator {
 public:
  explicit ConfiguredDemoLocator(DemoScene scene) : scene_(std::move(scene)) {}
  std::string resource_id() const override { return "configured_demo_scene"; }
  std::optional<Observation> locate(const std::string& id, Time now) override {
    try { return Observation{id, scene_.frame(), scene_.at(id).pose, now, true}; }
    catch (const std::out_of_range&) { return std::nullopt; }
  }
 private:
  DemoScene scene_;
};

struct TaskDefinition {
  std::string name;
  std::string template_id;
  std::string implementation;
};

// Only installed, reviewed templates are selectable. This is not a planner or
// an interpreter for client-supplied XML. Task aliases cannot select file paths.
class TaskCatalog {
 public:
  explicit TaskCatalog(std::vector<TaskDefinition> definitions) {
    if (definitions.empty()) throw std::invalid_argument("empty task catalog");
    for (auto& task : definitions) {
      if (!valid_id(task.name) || !valid_id(task.implementation) ||
          (task.template_id != "pick_place" && task.template_id != "locate_object"))
        throw std::invalid_argument("invalid task definition: " + task.name);
      const auto name = task.name;
      if (!definitions_.emplace(name, std::move(task)).second)
        throw std::invalid_argument("duplicate task: " + name);
    }
  }
  const std::map<std::string, TaskDefinition>& definitions() const { return definitions_; }
  static std::vector<std::string> required_inputs(const TaskDefinition& task) {
    return task.template_id == "pick_place" ? std::vector<std::string>{"object", "target"} :
                                             std::vector<std::string>{"object"};
  }
  const TaskDefinition& admit(const std::string& name, const std::string& object,
                            const std::string& target, const DemoScene& scene,
                            const Skills& skills, const Bindings& bindings) const {
    const auto& task = definitions_.at(name);
    if (!valid_id(object)) throw std::invalid_argument("invalid object ID");
    const auto& entity = scene.at(object);
    if (task.template_id == "pick_place") {
      if (!valid_id(target) || entity.role != EntityRole::object ||
          scene.at(target).role != EntityRole::target)
        throw std::invalid_argument("object and target roles do not match task");
      check_step(skills, bindings, task, "locate_object", {{"object", object}});
      check_step(skills, bindings, task, "pick_object", {{"object", object}});
      check_step(skills, bindings, task, "locate_object", {{"object", target}});
      check_step(skills, bindings, task, "place_object", {{"object", object}, {"target", target}});
    } else {
      if (!target.empty()) throw std::invalid_argument("locate task takes no target");
      check_step(skills, bindings, task, "locate_object", {{"object", object}});
    }
    return task;
  }
  void validate_configuration(const DemoScene& scene, const Skills& skills, const Bindings& bindings) const {
    for (const auto& entry : definitions_) {
      const auto& task = entry.second;
      skills.validate_dependencies("locate_object", task.implementation, bindings);
      if (task.template_id == "pick_place") {
        if (scene.ids(EntityRole::object).empty() || scene.ids(EntityRole::target).empty())
          throw std::invalid_argument("pick_place requires configured objects and targets");
        skills.validate_dependencies("pick_object", task.implementation, bindings);
        skills.validate_dependencies("place_object", task.implementation, bindings);
      }
    }
  }
 private:
  static void check_step(const Skills& skills, const Bindings& bindings, const TaskDefinition& task,
                         const std::string& id, const Arguments& args) {
    skills.validate_arguments(id, args);
    skills.validate_dependencies(id, task.implementation, bindings);
  }
  std::map<std::string, TaskDefinition> definitions_;
};
}  // namespace robot_core
