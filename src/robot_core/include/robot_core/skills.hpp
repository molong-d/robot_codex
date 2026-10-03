#pragma once
#include "robot_core/components.hpp"

namespace robot_core {
class Locate final : public Skill {
 public:
  explicit Locate(Context& c) : locator_(c.bindings.get<ObjectLocator>("perception")), world_(c.world) {}
  Result start(const Arguments& args, Time now) override {
    const auto object = args.at("object");
    world_.observations.erase(object);
    auto observation = locator_->locate(object, now);
    if (!observation || !observation->valid || observation->object_id != object ||
        observation->frame_id.empty() || !valid_pose(observation->pose) ||
        !fresh(observation->stamp, now, std::chrono::milliseconds(2000)))
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
  Manipulate(Context& c, bool pick, ManipulationPolicy policy = {})
      : arm_(c.bindings.get<ArmMotion>("motion")), gripper_(c.bindings.get<Gripper>("gripper")),
        world_(c.world), pick_(pick), policy_(std::move(policy)) {
    if (policy_.frame_id.empty() || !valid_tolerance(policy_.tolerance) ||
        policy_.observation_max_age.count() <= 0 || policy_.feedback_max_age.count() <= 0 ||
        !std::isfinite(policy_.grasp.width_m) || policy_.grasp.width_m < 0.0 ||
        !std::isfinite(policy_.grasp.max_effort_n) || policy_.grasp.max_effort_n <= 0.0 ||
        !std::isfinite(policy_.release_width_m) || policy_.release_width_m <= policy_.grasp.width_m ||
        !std::isfinite(policy_.gripper_tolerance_m) || policy_.gripper_tolerance_m <= 0.0)
      throw std::invalid_argument("invalid manipulation policy");
  }
  Result start(const Arguments& args, Time now) override {
    object_ = args.at("object");
    const auto observation_id = pick_ ? object_ : args.at("target");
    // Refresh/read measured initial gripper state before any arm command.
    gripper_->poll(now);
    const auto g = gripper_->feedback();
    if (!gripper_valid(g, now) || !g.stopped)
      return {Status::failed, "GRIPPER_NOT_READY", "fresh stationary gripper feedback required"};
    if (pick_) {
      if (g.grasp_detected || !world_.attached_object.empty())
        return {Status::failed, "GRIPPER_OCCUPIED", "gripper must be empty"};
    } else {
      target_id_ = observation_id;
      if (!g.grasp_detected || world_.attached_object != object_)
        return {Status::failed, "NOT_HOLDING", "requested object not held"};
    }
    const auto it = world_.observations.find(observation_id);
    if (it == world_.observations.end() || !it->second.valid ||
        it->second.object_id != observation_id || it->second.frame_id != policy_.frame_id ||
        !valid_pose(it->second.pose) || !fresh(it->second.stamp, now, policy_.observation_max_age))
      return {Status::failed, "STALE_OBSERVATION", "locate a valid motion target before manipulation"};
    target_ = {it->second.object_id, it->second.frame_id, it->second.pose, policy_.tolerance};
    world_.observations.erase(observation_id);
    // Set phase before dispatch so a throwing adapter can still receive best-effort stop.
    phase_ = Phase::moving;
    arm_->begin_move(target_);
    return {Status::running, "", "arm motion started"};
  }
  Result tick(Time now) override {
    if (phase_ == Phase::idle || phase_ == Phase::done) return result_;
    const auto status = phase_ == Phase::moving ? arm_->poll(now) : gripper_->poll(now);
    if (status == Status::faulted)
      return {Status::faulted, "COMPONENT_FAULT", "component state unknown"};
    if (!terminal(status))
      return {stopping_ ? Status::canceling : Status::running, "", "awaiting component completion"};
    // A server result (including canceled) is insufficient without current stop feedback.
    if (phase_ == Phase::moving) {
      const auto f = arm_->feedback();
      if (!f.valid || !valid_pose(f.pose) || !fresh(f.stamp, now, policy_.feedback_max_age) || !f.stopped)
        return {stopping_ ? Status::canceling : Status::running, "STOP_UNCONFIRMED", "awaiting fresh stationary arm feedback"};
      if (stopping_) return complete({Status::canceled, "CANCELED", "arm stopped; no next action dispatched"});
      if (status != Status::succeeded)
        return complete({Status::failed, "MOTION_FAILED", "arm action failed or canceled"});
      if (f.frame_id != target_.frame_id || !pose_near(f.pose, target_.pose, target_.tolerance))
        return complete({Status::failed, "MOTION_VERIFICATION_FAILED", "measured pose outside tolerance"});
      phase_ = Phase::gripping;
      if (pick_) gripper_->begin_grasp(policy_.grasp);
      else gripper_->begin_release(policy_.release_width_m);
      return {Status::running, "", pick_ ? "grasp started" : "release started"};
    }
    const auto g = gripper_->feedback();
    if (!gripper_valid(g, now) || !g.stopped)
      return {stopping_ ? Status::canceling : Status::running, "STOP_UNCONFIRMED", "awaiting fresh stationary gripper feedback"};
    // Reconcile completed grasp/release even if cancellation won the ROS result race.
    if (g.grasp_detected && pick_) {
      world_.attached_object = object_;
      world_.known_locations[object_] = "gripper";
    } else if (!g.grasp_detected && !pick_) {
      world_.attached_object.clear();
      world_.known_locations.erase(object_);
      world_.placement_candidates[object_] = target_id_;
    }
    if (stopping_) return complete({Status::canceled, "CANCELED", "gripper stopped; state reconciled"});
    if (status != Status::succeeded)
      return complete({Status::failed, "GRIPPER_FAILED", "gripper action failed or canceled"});
    const bool verified = pick_ ? g.grasp_detected :
        !g.grasp_detected && std::abs(g.width_m-policy_.release_width_m) <= policy_.gripper_tolerance_m;
    if (!verified)
      return complete({Status::failed, "GRIPPER_VERIFICATION_FAILED", "gripper outcome not observed"});
    return complete({Status::succeeded, "", pick_ ? "grasp detected" : "release verified; placement remains inferred"});
  }
  void cancel() override {
    if (stopping_ || phase_ == Phase::idle || phase_ == Phase::done) return;
    stopping_ = true;  // irreversible for this skill instance
    if (phase_ == Phase::moving) arm_->request_stop();
    else gripper_->request_stop();
  }
 private:
  bool gripper_valid(const GripperFeedback& g, Time now) const {
    return g.valid && std::isfinite(g.width_m) && g.width_m >= 0.0 &&
           fresh(g.stamp, now, policy_.feedback_max_age);
  }
  Result complete(Result r) { phase_ = Phase::done; return result_ = std::move(r); }
  enum class Phase { idle, moving, gripping, done };
  std::shared_ptr<ArmMotion> arm_;
  std::shared_ptr<Gripper> gripper_;
  WorldState& world_;
  bool pick_, stopping_{false};
  ManipulationPolicy policy_;
  Phase phase_{Phase::idle};
  Result result_;
  std::string object_, target_id_;
  CartesianTarget target_;
};

inline void register_demo_skills(Skills& skills, ManipulationPolicy policy = {}) {
  skills.define({"locate_object", "Obtain a fresh object pose", {"object"},
      "camera ready", "observation valid", "fresh object pose in a known frame"});
  skills.define({"pick_object", "Move to and grasp a previously located object", {"object"},
      "empty gripper and fresh object pose", "exclusive arm and gripper control", "grasp detected"});
  skills.define({"place_object", "Move to a located target and release the held object", {"object", "target"},
      "requested object held and fresh target pose", "exclusive arm and gripper control", "release detected"});
  skills.implement("locate_object", "standard", [](Context& c) { return std::make_unique<Locate>(c); },
      {{{"perception", "object_locator", 1}}, {}, ""});
  const Dependencies manipulation{
      {{"motion", "arm_motion", 2}, {"gripper", "gripper", 2}, {"safety", "execution_gate", 1}},
      {"motion", "gripper"}, "safety"};
  skills.implement("pick_object", "standard", [policy](Context& c) { return std::make_unique<Manipulate>(c, true, policy); }, manipulation);
  skills.implement("place_object", "standard", [policy](Context& c) { return std::make_unique<Manipulate>(c, false, policy); }, manipulation);
}
}  // namespace robot_core
