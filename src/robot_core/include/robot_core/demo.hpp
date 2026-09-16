#pragma once
#include "robot_core/core.hpp"
#include <optional>

namespace robot_core {

// Perception component: produces a time-stamped pose observation.
class ObjectLocator : public Component {
 public:
  std::string interface_id() const final { return "object_locator"; }
  virtual std::optional<Observation> locate(const std::string&, Time) = 0;
};

// Motion component: a MoveIt adapter can implement this contract. ros2_control
// remains below it as the controller/hardware execution framework.
struct CartesianTarget {
  std::string id;
  std::string frame_id;
  Pose pose;
};
class ArmMotion : public Component {
 public:
  std::string interface_id() const final { return "arm_motion"; }
  virtual void begin_move(const CartesianTarget&) = 0;
  virtual Status poll() = 0;
  virtual void request_stop() = 0;
  virtual bool reached(const CartesianTarget&) const = 0;
};

// End-effector component: uses physical width/effort, not object names.
struct GraspCommand {
  double width_m{0.0};
  double max_effort_n{0.0};
};
class Gripper : public Component {
 public:
  std::string interface_id() const final { return "gripper"; }
  virtual void begin_grasp(const GraspCommand&) = 0;
  virtual void begin_release(double width_m) = 0;
  virtual Status poll() = 0;
  virtual void request_stop() = 0;
  virtual bool grasp_detected() const = 0;
};

class MockExecutionGate final : public ExecutionGate {
 public:
  explicit MockExecutionGate(bool permitted = true) : permitted_(permitted) {}
  std::string resource_id() const override { return "demo_safety_system"; }
  Admission admit(const std::string& skill, const Arguments&, Time) const override {
    if (permitted_) return {true, "", "mock execution permitted"};
    return {false, "SAFETY_INTERLOCK", "mock execution gate is not armed for " + skill};
  }
 private:
  bool permitted_;
};

class MockLocator final : public ObjectLocator {
 public:
  std::string resource_id() const override { return "demo_camera"; }
  std::optional<Observation> locate(const std::string& object, Time now) override {
    if (object == "workpiece")
      return Observation{object, "base_link", {0.40, 0.10, 0.20, 0.0, 0.0, 0.0, 1.0}, now, true};
    if (object == "tray")
      return Observation{object, "base_link", {0.60, -0.20, 0.15, 0.0, 0.0, 0.0, 1.0}, now, true};
    return std::nullopt;
  }
};

struct MockRobotState {
  std::string arm_target;
  bool grasped{false};
};

class MockArmMotion final : public ArmMotion {
 public:
  MockArmMotion(std::shared_ptr<MockRobotState> state, int ticks = 3, bool fail = false)
      : state_(std::move(state)), duration_(ticks), fail_(fail) {
    if (!state_ || ticks <= 0) throw std::invalid_argument("invalid mock arm configuration");
  }
  std::string resource_id() const override { return "demo_arm"; }
  void begin_move(const CartesianTarget& target) override {
    if (busy_ || target.id.empty() || target.frame_id.empty())
      throw std::runtime_error("invalid mock arm move");
    target_ = target;
    busy_ = true;
    stopping_ = false;
    remaining_ = duration_;
    last_ = Status::running;
  }
  Status poll() override {
    if (!busy_) return last_;
    if (stopping_) {
      if (--remaining_ > 0) return Status::canceling;
      busy_ = false;
      return last_ = Status::canceled;
    }
    if (--remaining_ > 0) return Status::running;
    busy_ = false;
    if (fail_) return last_ = Status::failed;
    state_->arm_target = target_.id;
    return last_ = Status::succeeded;
  }
  void request_stop() override {
    if (busy_ && !stopping_) { stopping_ = true; remaining_ = 2; }
  }
  bool reached(const CartesianTarget& target) const override {
    return state_->arm_target == target.id;
  }
 private:
  std::shared_ptr<MockRobotState> state_;
  CartesianTarget target_;
  int duration_{0}, remaining_{0};
  bool fail_{false}, busy_{false}, stopping_{false};
  Status last_{Status::idle};
};

class MockGripper final : public Gripper {
 public:
  MockGripper(std::shared_ptr<MockRobotState> state, int ticks = 2, bool fail_grasp = false)
      : state_(std::move(state)), duration_(ticks), fail_grasp_(fail_grasp) {
    if (!state_ || ticks <= 0) throw std::invalid_argument("invalid mock gripper configuration");
  }
  std::string resource_id() const override { return "demo_gripper"; }
  void begin_grasp(const GraspCommand& command) override {
    if (busy_ || state_->grasped || command.width_m < 0.0 || command.max_effort_n <= 0.0)
      throw std::runtime_error("invalid mock grasp");
    closing_ = true;
    begin();
  }
  void begin_release(double width_m) override {
    if (busy_ || !state_->grasped || width_m <= 0.0)
      throw std::runtime_error("invalid mock release");
    closing_ = false;
    begin();
  }
  Status poll() override {
    if (!busy_) return last_;
    if (stopping_) {
      if (--remaining_ > 0) return Status::canceling;
      busy_ = false;
      return last_ = Status::canceled;
    }
    if (--remaining_ > 0) return Status::running;
    busy_ = false;
    if (closing_ && fail_grasp_) return last_ = Status::failed;
    state_->grasped = closing_;
    return last_ = Status::succeeded;
  }
  void request_stop() override {
    if (busy_ && !stopping_) { stopping_ = true; remaining_ = 2; }
  }
  bool grasp_detected() const override { return state_->grasped; }
 private:
  void begin() { busy_ = true; stopping_ = false; remaining_ = duration_; last_ = Status::running; }
  std::shared_ptr<MockRobotState> state_;
  int duration_{0}, remaining_{0};
  bool fail_grasp_{false}, busy_{false}, stopping_{false}, closing_{false};
  Status last_{Status::idle};
};

class Locate final : public Skill {
 public:
  explicit Locate(Context& c) : locator_(c.bindings.get<ObjectLocator>("perception")), world_(c.world) {}
  Result start(const Arguments& args, Time now) override {
    const auto object = args.at("object");
    world_.observations.erase(object);
    auto observation = locator_->locate(object, now);
    if (!observation || !observation->valid || observation->object_id != object ||
        observation->frame_id.empty() || observation->stamp > now ||
        now - observation->stamp > std::chrono::seconds(2))
      return result_ = {Status::failed, "INVALID_OBSERVATION", "no fresh, valid pose observation"};
    world_.observations[object] = *observation;
    return result_ = {Status::succeeded, "", "fresh pose observation stored"};
  }
  Result tick(Time) override { return result_; }
  void cancel() override {}
 private:
  std::shared_ptr<ObjectLocator> locator_;
  WorldState& world_;
  Result result_;
};

class Manipulate final : public Skill {
 public:
  Manipulate(Context& c, bool pick)
      : arm_(c.bindings.get<ArmMotion>("motion")), gripper_(c.bindings.get<Gripper>("gripper")),
        world_(c.world), pick_(pick) {}
  Result start(const Arguments& args, Time now) override {
    object_ = args.at("object");
    const auto observation_id = pick_ ? object_ : args.at("target");
    if (pick_) {
      if (gripper_->grasp_detected() || !world_.attached_object.empty())
        return {Status::failed, "GRIPPER_OCCUPIED", "gripper must be empty"};
    } else {
      target_id_ = observation_id;
      if (!gripper_->grasp_detected() || world_.attached_object != object_)
        return {Status::failed, "NOT_HOLDING", "requested object not held"};
    }
    const auto it = world_.observations.find(observation_id);
    if (it == world_.observations.end() || !it->second.valid ||
        it->second.object_id != observation_id || it->second.frame_id != "base_link" ||
        it->second.stamp > now || now - it->second.stamp > std::chrono::seconds(2))
      return {Status::failed, "STALE_OBSERVATION", "locate motion target before manipulation"};
    target_ = {it->second.object_id, it->second.frame_id, it->second.pose};
    world_.observations.erase(observation_id);
    arm_->begin_move(target_);
    phase_ = Phase::moving;
    return {Status::running, "", "arm motion started"};
  }
  Result tick(Time) override {
    if (phase_ == Phase::moving) {
      const auto status = arm_->poll();
      if (status != Status::succeeded)
        return {status, status == Status::failed ? "MOTION_FAILED" : "", name(status)};
      if (!arm_->reached(target_))
        return {Status::failed, "MOTION_VERIFICATION_FAILED", "arm target not observed"};
      if (pick_) gripper_->begin_grasp({0.01, 20.0});
      else gripper_->begin_release(0.08);
      phase_ = Phase::gripping;
      return {Status::running, "", pick_ ? "grasp started" : "release started"};
    }
    const auto status = gripper_->poll();
    if (status != Status::succeeded)
      return {status, status == Status::failed ? "GRIPPER_FAILED" : "", name(status)};
    const bool verified = pick_ ? gripper_->grasp_detected() : !gripper_->grasp_detected();
    if (!verified)
      return {Status::failed, "GRIPPER_VERIFICATION_FAILED", "gripper outcome not observed"};
    if (pick_) {
      world_.attached_object = object_;
      world_.known_locations[object_] = "gripper";
    } else {
      world_.attached_object.clear();
      world_.known_locations[object_] = target_id_;
    }
    phase_ = Phase::done;
    return {Status::succeeded, "", pick_ ? "object grasp verified" : "release verified"};
  }
  void cancel() override {
    if (phase_ == Phase::moving) arm_->request_stop();
    else if (phase_ == Phase::gripping) gripper_->request_stop();
  }
 private:
  enum class Phase { idle, moving, gripping, done };
  std::shared_ptr<ArmMotion> arm_;
  std::shared_ptr<Gripper> gripper_;
  WorldState& world_;
  bool pick_;
  Phase phase_{Phase::idle};
  std::string object_, target_id_;
  CartesianTarget target_;
};

inline void register_demo_skills(Skills& skills) {
  skills.define({"locate_object", "Obtain a fresh object pose", {"object"},
      "camera ready", "observation valid", "fresh object pose in a known frame"});
  skills.define({"pick_object", "Move to and grasp a previously located object", {"object"},
      "empty gripper and fresh object pose", "exclusive arm and gripper control", "grasp detected"});
  skills.define({"place_object", "Move to a located target and release the held object", {"object", "target"},
      "requested object held and fresh target pose", "exclusive arm and gripper control",
      "release detected and object assigned to target"});
  skills.implement("locate_object", "standard", [](Context& c) { return std::make_unique<Locate>(c); },
      {{{"perception", "object_locator", 1}}, {}, ""});
  const Dependencies manipulation{
      {{"motion", "arm_motion", 1}, {"gripper", "gripper", 1}, {"safety", "execution_gate", 1}},
      {"motion", "gripper"}, "safety"};
  skills.implement("pick_object", "standard", [](Context& c) { return std::make_unique<Manipulate>(c, true); },
      manipulation);
  skills.implement("place_object", "standard", [](Context& c) { return std::make_unique<Manipulate>(c, false); },
      manipulation);
}
}  // namespace robot_core
