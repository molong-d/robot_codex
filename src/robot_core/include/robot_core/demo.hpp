#pragma once
#include "robot_core/skills.hpp"

namespace robot_core {
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
      return Observation{object, "base_link", {0.40, 0.10, 0.20, 0.0, 0.0, 0.0, 1.0}, now, true,
                         {"mock_locator", true, 1.0}, PoseMeaning::motion_target};
    if (object == "tray")
      return Observation{object, "base_link", {0.60, -0.20, 0.15, 0.0, 0.0, 0.0, 1.0}, now, true,
                         {"mock_locator", true, 1.0}, PoseMeaning::motion_target};
    return std::nullopt;
  }
};

struct MockRobotState {
  std::string arm_target;
  bool grasped{false};
  Pose arm_pose;
  double width_m{0.08};
};

class MockArmMotion final : public ArmMotion {
 public:
  MockArmMotion(std::shared_ptr<MockRobotState> state, int ticks = 3, bool fail = false, int stop_ticks = 2)
      : state_(std::move(state)), duration_(ticks), stop_duration_(stop_ticks), fail_(fail) {
    if (!state_ || ticks <= 0 || stop_ticks <= 0) throw std::invalid_argument("invalid mock arm configuration");
  }
  std::string resource_id() const override { return "demo_arm"; }
  void begin_move(const CartesianTarget& target) override {
    if (busy_ || target.id.empty() || target.frame_id.empty() || !valid_pose(target.pose) || !valid_tolerance(target.tolerance))
      throw std::runtime_error("invalid mock arm move");
    target_ = target;
    busy_ = true;
    stopping_ = false;
    remaining_ = duration_;
    last_ = Status::running;
  }
  Status poll(Time now) override {
    feedback_.stamp = now;
    feedback_.valid = true;
    feedback_.stopped = !busy_;
    if (!busy_) return last_;
    if (stopping_) {
      if (--remaining_ > 0) return Status::canceling;
      busy_ = false;
      feedback_.stopped = true;
      return last_ = Status::canceled;
    }
    if (--remaining_ > 0) return Status::running;
    busy_ = false;
    feedback_.stopped = true;
    if (fail_) return last_ = Status::failed;
    state_->arm_target = target_.id;
    state_->arm_pose = target_.pose;
    feedback_.frame_id = target_.frame_id;
    feedback_.pose = target_.pose;
    return last_ = Status::succeeded;
  }
  void request_stop() override {
    if (busy_ && !stopping_) { stopping_ = true; remaining_ = stop_duration_; }
  }
  MotionFeedback feedback() const override { return feedback_; }
 private:
  std::shared_ptr<MockRobotState> state_;
  CartesianTarget target_;
  MotionFeedback feedback_{"base_link", {}, {}, false, true};
  int duration_{0}, stop_duration_{2}, remaining_{0};
  bool fail_{false}, busy_{false}, stopping_{false};
  Status last_{Status::idle};
};

class MockGripper final : public Gripper {
 public:
  MockGripper(std::shared_ptr<MockRobotState> state, int ticks = 2, bool fail_grasp = false, int stop_ticks = 2)
      : state_(std::move(state)), duration_(ticks), stop_duration_(stop_ticks), fail_grasp_(fail_grasp) {
    if (!state_ || ticks <= 0 || stop_ticks <= 0) throw std::invalid_argument("invalid mock gripper configuration");
  }
  std::string resource_id() const override { return "demo_gripper"; }
  void begin_grasp(const GraspCommand& command) override {
    if (busy_ || state_->grasped || !std::isfinite(command.width_m) || command.width_m < 0.0 ||
        !std::isfinite(command.max_effort_n) || command.max_effort_n <= 0.0)
      throw std::runtime_error("invalid mock grasp");
    commanded_width_ = command.width_m;
    closing_ = true;
    begin();
  }
  void begin_release(double width_m) override {
    if (busy_ || !state_->grasped || !std::isfinite(width_m) || width_m <= 0.0)
      throw std::runtime_error("invalid mock release");
    commanded_width_ = width_m;
    closing_ = false;
    begin();
  }
  Status poll(Time now) override {
    feedback_.stamp = now;
    feedback_.sample_id = static_cast<uint64_t>(now.time_since_epoch().count());
    feedback_.valid = true;
    feedback_.stopped = !busy_;
    if (!busy_) return last_;
    if (stopping_) {
      if (--remaining_ > 0) return Status::canceling;
      busy_ = false;
      feedback_.stopped = true;
      return last_ = Status::canceled;
    }
    if (--remaining_ > 0) return Status::running;
    busy_ = false;
    feedback_.stopped = true;
    if (closing_ && fail_grasp_) return last_ = Status::failed;
    state_->grasped = closing_;
    state_->width_m = commanded_width_;
    return last_ = Status::succeeded;
  }
  void request_stop() override {
    if (busy_ && !stopping_) { stopping_ = true; remaining_ = stop_duration_; }
  }
  GripperFeedback feedback() const override {
    auto f = feedback_;
    f.width_m = state_->width_m;
    f.grasp_detected = state_->grasped;
    return f;
  }
 private:
  void begin() { busy_ = true; stopping_ = false; remaining_ = duration_; last_ = Status::running; }
  std::shared_ptr<MockRobotState> state_;
  int duration_{0}, stop_duration_{2}, remaining_{0};
  GripperFeedback feedback_;
  double commanded_width_{0.08};
  bool fail_grasp_{false}, busy_{false}, stopping_{false}, closing_{false};
  Status last_{Status::idle};
};

}  // namespace robot_core
