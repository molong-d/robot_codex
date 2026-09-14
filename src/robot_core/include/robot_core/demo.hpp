#pragma once
#include "robot_core/core.hpp"
#include <optional>

namespace robot_core {
// Deliberately small reference contracts, not a universal robot/model schema.
class ObjectLocator : public Component {
 public:
  std::string interface_id() const final { return "object_locator"; }
  virtual std::optional<Observation> locate(const std::string&, Time) = 0;
};
class Manipulator : public Component {
 public:
  std::string interface_id() const final { return "manipulator"; }
  virtual void begin_pick(const std::string&) = 0;
  virtual void begin_place(const std::string&) = 0;
  virtual Status poll() = 0;
  virtual void request_stop() = 0;
  virtual std::string holding() const = 0;
  virtual std::string location(const std::string&) const = 0;
};

class MockLocator final : public ObjectLocator {
 public:
  std::string resource_id() const override { return "demo_camera"; }
  std::optional<Observation> locate(const std::string& object, Time now) override {
    if (object != "workpiece") return std::nullopt;
    return Observation{object, "base_link", now, true};
  }
};

class MockManipulator final : public Manipulator {
 public:
  explicit MockManipulator(int ticks = 3, bool fail_pick = false)
      : duration_(ticks), fail_pick_(fail_pick) {
    if (ticks <= 0) throw std::invalid_argument("mock ticks must be positive");
  }
  std::string resource_id() const override { return "demo_arm_and_gripper"; }
  void begin_pick(const std::string& object) override {
    if (busy_ || !held_.empty() || object != "workpiece")
      throw std::runtime_error("invalid mock pick");
    begin(); object_ = object; picking_ = true;
  }
  void begin_place(const std::string& target) override {
    if (busy_ || held_.empty() || target != "tray")
      throw std::runtime_error("invalid mock place");
    begin(); object_ = held_; target_ = target; picking_ = false;
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
    if (picking_ && fail_pick_) return last_ = Status::failed;
    if (picking_) { held_ = object_; locations_[object_] = "gripper"; }
    else { locations_[object_] = target_; held_.clear(); }
    return last_ = Status::succeeded;
  }
  void request_stop() override {
    if (busy_ && !stopping_) { stopping_ = true; remaining_ = 2; }
  }
  std::string holding() const override { return held_; }
  std::string location(const std::string& object) const override {
    const auto it = locations_.find(object);
    return it == locations_.end() ? "" : it->second;
  }
 private:
  void begin() { busy_ = true; stopping_ = false; remaining_ = duration_; last_ = Status::running; }
  int duration_, remaining_{0};
  bool fail_pick_, busy_{false}, stopping_{false}, picking_{false};
  Status last_{Status::idle};
  std::string held_, object_, target_;
  std::map<std::string, std::string> locations_{{"workpiece", "table"}};
};

class Locate final : public Skill {
 public:
  explicit Locate(Context& c) : locator_(c.bindings.get<ObjectLocator>("perception")), world_(c.world) {}
  Result start(const Arguments& args, Time now) override {
    const auto object = args.at("object");
    world_.observations.erase(object); // a failed refresh must not retain stale data
    auto observation = locator_->locate(object, now);
    if (!observation || !observation->valid || observation->object_id != object ||
        observation->frame_id.empty() || observation->stamp > now ||
        now - observation->stamp > std::chrono::seconds(2))
      return result_ = {Status::failed, "INVALID_OBSERVATION", "no fresh, valid observation"};
    world_.observations[object] = *observation;
    return result_ = {Status::succeeded, "", "object located (mock, no 6D estimate)"};
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
      : arm_(c.bindings.get<Manipulator>("motion")), world_(c.world), pick_(pick) {}
  Result start(const Arguments& args, Time now) override {
    object_ = args.at("object");
    if (pick_) {
      auto it = world_.observations.find(object_);
      if (!arm_->holding().empty()) return {Status::failed, "GRIPPER_OCCUPIED", "gripper must be empty"};
      if (it == world_.observations.end() || !it->second.valid ||
          it->second.object_id != object_ || it->second.frame_id != "base_link" ||
          it->second.stamp > now || now - it->second.stamp > std::chrono::seconds(2))
        return {Status::failed, "STALE_OBSERVATION", "locate object before picking"};
      // Geometry may change even if an attempted action fails or is canceled.
      world_.observations.erase(object_);
      arm_->begin_pick(object_);
    } else {
      target_ = args.at("target");
      if (target_ != "tray") return {Status::failed, "UNKNOWN_TARGET", "demo supports tray only"};
      if (arm_->holding() != object_) return {Status::failed, "NOT_HOLDING", "requested object not held"};
      arm_->begin_place(target_);
    }
    return {Status::running, "", "command accepted, completion not yet verified"};
  }
  Result tick(Time) override {
    auto status = arm_->poll();
    if (status == Status::succeeded) {
      const bool verified = pick_ ? arm_->holding() == object_ :
          arm_->holding().empty() && arm_->location(object_) == target_;
      if (!verified) return {Status::failed, "VERIFICATION_FAILED", "outcome not observed"};
    }
    return {status, status == Status::failed ? "EXECUTION_FAILED" : "", name(status)};
  }
  void cancel() override { arm_->request_stop(); }
 private:
  std::shared_ptr<Manipulator> arm_;
  WorldState& world_;
  bool pick_;
  std::string object_, target_;
};

inline void register_demo_skills(Skills& skills) {
  skills.define({"locate_object", "Obtain a fresh object observation", {"object"},
      "camera ready", "observation valid",
      "fresh object observation in a known frame"});
  skills.define({"pick_object", "Pick a previously located object", {"object"},
      "empty gripper and fresh observation",
      "exclusive arm control", "requested object is held"});
  skills.define({"place_object", "Place the held object at a known target", {"object", "target"},
      "requested object is held",
      "exclusive arm control", "object at target and gripper empty"});
  skills.implement("locate_object", "standard", [](Context& c) { return std::make_unique<Locate>(c); },
      {{{"perception", "object_locator", 1}}, {}});
  skills.implement("pick_object", "standard", [](Context& c) { return std::make_unique<Manipulate>(c, true); },
      {{{"motion", "manipulator", 1}}, {"motion"}});
  skills.implement("place_object", "standard", [](Context& c) { return std::make_unique<Manipulate>(c, false); },
      {{{"motion", "manipulator", 1}}, {"motion"}});
}
}  // namespace robot_core
