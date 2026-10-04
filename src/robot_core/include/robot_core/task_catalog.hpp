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
  explicit ConfiguredDemoLocator(DemoScene scene, PoseMeaning meaning = PoseMeaning::motion_target,
                                std::string source = "configured_demo")
      : scene_(std::move(scene)), meaning_(meaning), source_(std::move(source)) {
    if (!valid_id(source_) || (meaning_ != PoseMeaning::object_pose && meaning_ != PoseMeaning::motion_target))
      throw std::invalid_argument("invalid configured perception semantics");
  }
  std::string resource_id() const override { return "configured_demo_scene"; }
  std::optional<Observation> locate(const std::string& id, Time now) override {
    try { return Observation{id, scene_.frame(), scene_.at(id).pose, now, true,
                             {source_, true, 1.0}, meaning_}; }
    catch (const std::out_of_range&) { return std::nullopt; }
  }
 private:
  DemoScene scene_;
  PoseMeaning meaning_;
  std::string source_;
};

// Deliberately synthetic: robot feedback plus configured end-effector targets,
// NOT independent object/contact sensing. Never reads placement_candidates.
class DemoOutcomeObserver final : public ManipulationObserver {
 public:
  DemoOutcomeObserver(std::shared_ptr<ArmMotion> arm, std::shared_ptr<Gripper> gripper,
                      DemoScene scene, std::string failure = "none")
      : arm_(std::move(arm)), gripper_(std::move(gripper)), scene_(std::move(scene)), failure_(std::move(failure)) {
    if (!arm_ || !gripper_ || (failure_ != "none" && failure_ != "grasp" && failure_ != "placement"))
      throw std::invalid_argument("invalid demo outcome observer");
  }
  std::string resource_id() const override { return "demo_outcome_feedback"; }
  std::optional<OutcomeEvidence> grasp(const std::string& object, Time now) override {
    return observe(object, "", now);
  }
  std::optional<OutcomeEvidence> placement(const std::string& object, const std::string& target, Time now) override {
    return observe(object, target, now);
  }
 private:
  std::optional<OutcomeEvidence> observe(const std::string& object, const std::string& target, Time now) {
    // Refresh cached measurements/status; poll never dispatches a new command.
    arm_->poll(now); gripper_->poll(now);
    const auto arm = arm_->feedback(); const auto gripper = gripper_->feedback();
    try {
      const auto& pose = scene_.at(target.empty() ? object : target).pose;
      const bool condition = arm.stopped && gripper.stopped && arm.frame_id == scene_.frame() &&
          pose_near(arm.pose, pose, {}) &&
          (target.empty() ? dual_finger_grasp(gripper.contact_state) :
              no_finger_contact(gripper.contact_state) && std::abs(gripper.width_m-0.08) <= 0.002) &&
          failure_ != (target.empty() ? "grasp" : "placement");
      return OutcomeEvidence{object, target, std::min(arm.stamp, gripper.stamp),
          arm.valid && gripper.valid, condition, {"demo_outcome", true, 1.0}, std::min(arm.sample_id, gripper.sample_id)};
    } catch (const std::out_of_range&) { return std::nullopt; }
  }
  std::shared_ptr<ArmMotion> arm_;
  std::shared_ptr<Gripper> gripper_;
  DemoScene scene_;
  std::string failure_;
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
          (task.template_id != "pick_place" && task.template_id != "locate_object" && task.template_id != "verified_pick_place"))
        throw std::invalid_argument("invalid task definition: " + task.name);
      const auto name = task.name;
      if (!definitions_.emplace(name, std::move(task)).second)
        throw std::invalid_argument("duplicate task: " + name);
    }
  }
  const std::map<std::string, TaskDefinition>& definitions() const { return definitions_; }
  static std::vector<std::string> required_inputs(const TaskDefinition& task) {
    return task.template_id != "locate_object" ? std::vector<std::string>{"object", "target"} :
                                             std::vector<std::string>{"object"};
  }
  static uint32_t step_count(const TaskDefinition& task) {
    return task.template_id == "verified_pick_place" ? 6 : task.template_id == "pick_place" ? 4 : 1;
  }
  const TaskDefinition& admit(const std::string& name, const std::string& object,
                            const std::string& target, const DemoScene& scene,
                            const Skills& skills, const Bindings& bindings) const {
    const auto& task = definitions_.at(name);
    if (!valid_id(object)) throw std::invalid_argument("invalid object ID");
    const auto& entity = scene.at(object);
    if (task.template_id != "locate_object") {
      if (!valid_id(target) || entity.role != EntityRole::object ||
          scene.at(target).role != EntityRole::target)
        throw std::invalid_argument("object and target roles do not match task");
      check_step(skills, bindings, task, "locate_object", {{"object", object}});
      check_step(skills, bindings, task, "pick_object", {{"object", object}});
      check_step(skills, bindings, task, "locate_object", {{"object", target}});
      check_step(skills, bindings, task, "place_object", {{"object", object}, {"target", target}});
      if (task.template_id == "verified_pick_place") {
        check_step(skills, bindings, task, "verify_grasp", {{"object", object}});
        check_step(skills, bindings, task, "verify_placement", {{"object", object}, {"target", target}});
      }
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
      if (task.template_id != "locate_object") {
        if (scene.ids(EntityRole::object).empty() || scene.ids(EntityRole::target).empty())
          throw std::invalid_argument("pick_place requires configured objects and targets");
        skills.validate_dependencies("pick_object", task.implementation, bindings);
        skills.validate_dependencies("place_object", task.implementation, bindings);
        if (task.template_id == "verified_pick_place") {
          skills.validate_dependencies("verify_grasp", task.implementation, bindings);
          skills.validate_dependencies("verify_placement", task.implementation, bindings);
        }
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
