#pragma once
#include "robot_core/components.hpp"
#include "robot_core/verification.hpp"

namespace robot_core {
class Locate final : public Skill {
 public:
  explicit Locate(Context& c, EvidencePolicy policy = {std::chrono::milliseconds(2000), 0.8, false})
      : locator_(c.bindings.get<ObjectLocator>("perception")), world_(c.world), policy_(policy) {
    if (!valid_evidence_policy(policy_)) throw std::invalid_argument("invalid locate evidence policy");
  }
  Result start(const Arguments& args, Time now) override {
    const auto object = args.at("object");
    world_.observations.erase(object);
    auto observation = locator_->locate(object, now);
    if (!observation || !observation->valid || observation->object_id != object ||
        observation->frame_id.empty() || !valid_pose(observation->pose) ||
        (observation->meaning != PoseMeaning::object_pose && observation->meaning != PoseMeaning::motion_target) ||
        !acceptable_evidence(observation->evidence, observation->stamp, now, policy_))
      return result_ = {Status::failed, "INVALID_OBSERVATION", "no fresh valid pose with accepted source and quality"};
    world_.observations[object] = *observation;
    return result_ = {Status::succeeded, "", "fresh pose observation stored"};
  }
  Result tick(Time) override { return result_; }
  void cancel() override {}
 private:
  std::shared_ptr<ObjectLocator> locator_;
  WorldState& world_;
  EvidencePolicy policy_;
  Result result_;
};

class Manipulate final : public Skill {
 public:
  Manipulate(Context& c, bool pick, ManipulationPolicy policy = {}, bool resolve_target = false)
      : arm_(c.bindings.get<ArmMotion>("motion")), gripper_(c.bindings.get<Gripper>("gripper")),
        world_(c.world), pick_(pick), policy_(std::move(policy)) {
    if (resolve_target) resolver_ = c.bindings.get<MotionTargetResolver>("target_resolver");
    if (policy_.frame_id.empty() || policy_.tool_frame.empty() || !valid_tolerance(policy_.tolerance) ||
        !valid_evidence_policy({policy_.observation_max_age, policy_.minimum_observation_quality, policy_.allow_synthetic}) ||
        policy_.feedback_max_age.count() <= 0 ||
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
        it->second.object_id != observation_id || it->second.frame_id.empty() ||
        !valid_pose(it->second.pose) || !acceptable_evidence(it->second.evidence, it->second.stamp, now,
            {policy_.observation_max_age, policy_.minimum_observation_quality, policy_.allow_synthetic}))
      return {Status::failed, "STALE_OBSERVATION", "locate a valid motion target before manipulation"};
    auto motion_target = it->second;
    const auto source_frame = motion_target.frame_id;
    std::string calibration;
    if (resolver_) {
      if (motion_target.meaning != PoseMeaning::object_pose)
        return {Status::failed, "INVALID_SOURCE_POSE", "pose_resolved requires a native object pose; never apply offsets twice"};
      std::optional<ResolvedMotionTarget> resolved;
      try { resolved = resolver_->resolve(motion_target, pick_ ? TargetPurpose::grasp : TargetPurpose::placement); }
      catch (const std::exception&) {
        return {Status::failed, "TARGET_RESOLUTION_FAILED", "target resolver failed before motion dispatch"};
      }
      if (!resolved || !valid_id(resolved->calibration_id) || resolved->tool_frame != policy_.tool_frame ||
          !resolved->observation.valid || resolved->observation.object_id != observation_id ||
          resolved->observation.frame_id != policy_.frame_id || resolved->observation.meaning != PoseMeaning::motion_target ||
          !valid_pose(resolved->observation.pose) || resolved->observation.stamp != motion_target.stamp ||
          resolved->observation.evidence.source != motion_target.evidence.source ||
          resolved->observation.evidence.quality != motion_target.evidence.quality ||
          (motion_target.evidence.synthetic && !resolved->observation.evidence.synthetic) ||
          !acceptable_evidence(resolved->observation.evidence, resolved->observation.stamp, now,
              {policy_.observation_max_age, policy_.minimum_observation_quality, policy_.allow_synthetic}))
        return {Status::failed, "TARGET_RESOLUTION_FAILED", "resolved target has incompatible frame/tool or altered evidence"};
      calibration = resolved->calibration_id;
      motion_target = resolved->observation;
    } else {
      if (motion_target.frame_id != policy_.frame_id)
        return {Status::failed, "STALE_OBSERVATION", "motion target must already be in the configured base frame"};
      if (motion_target.meaning != PoseMeaning::motion_target)
        return {Status::failed, "TARGET_RESOLUTION_REQUIRED", "object pose needs an explicit calibrated motion-target transform"};
    }
    target_ = {motion_target.object_id, motion_target.frame_id, motion_target.pose, policy_.tolerance};
    world_.observations.erase(observation_id);
    world_.grasp_verifications.erase(object_);
    world_.placement_verifications.erase(object_);
    if (pick_) {
      world_.known_locations.erase(object_);
      world_.placement_candidates.erase(object_);
      world_.release_stamps.erase(object_);
    }
    // Set phase before dispatch so a throwing adapter can still receive best-effort stop.
    phase_ = Phase::moving;
    arm_->begin_move(target_);
    return {Status::running, "", calibration.empty() ? "arm motion started" :
        "arm motion started; calibration=" + calibration + "; source_frame=" + source_frame + "; tool=" + policy_.tool_frame};
  }
  Result tick(Time now) override {
    if (phase_ == Phase::idle || phase_ == Phase::done) return result_;
    const auto status = phase_ == Phase::moving ? arm_->poll(now) : gripper_->poll(now);
    if (status == Status::faulted)
      return {Status::faulted, "COMPONENT_FAULT", "component state unknown"};
    if (!terminal(status))
      return {stopping_ ? Status::canceling : Status::running, "", "awaiting component completion"};
    if (terminal_since_ == Time{}) terminal_since_ = now;
    // A server result (including canceled) is insufficient without current stop feedback.
    if (phase_ == Phase::moving) {
      const auto f = arm_->feedback();
      if (!f.valid || !valid_pose(f.pose) || !fresh(f.stamp, now, policy_.feedback_max_age) ||
          f.stamp < terminal_since_ || !f.stopped)
        return {stopping_ ? Status::canceling : Status::running, "STOP_UNCONFIRMED", "awaiting fresh stationary arm feedback"};
      if (stopping_) return complete({Status::canceled, "CANCELED", "arm stopped; no next action dispatched"});
      if (status != Status::succeeded)
        return complete({Status::failed, "MOTION_FAILED", "arm action failed or canceled"});
      if (f.frame_id != target_.frame_id || !pose_near(f.pose, target_.pose, target_.tolerance))
        return complete({Status::failed, "MOTION_VERIFICATION_FAILED", "measured pose outside tolerance"});
      phase_ = Phase::gripping;
      terminal_since_ = {};
      if (pick_) gripper_->begin_grasp(policy_.grasp);
      else gripper_->begin_release(policy_.release_width_m);
      return {Status::running, "", pick_ ? "grasp started" : "release started"};
    }
    const auto g = gripper_->feedback();
    if (!gripper_valid(g, now) || g.stamp < terminal_since_ || !g.stopped)
      return {stopping_ ? Status::canceling : Status::running, "STOP_UNCONFIRMED", "awaiting fresh stationary gripper feedback"};
    // Reconcile completed grasp/release even if cancellation won the ROS result race.
    if (g.grasp_detected && pick_) {
      world_.attached_object = object_;
      world_.attachment_stamp = g.stamp;
      world_.known_locations[object_] = "gripper";
    } else if (!g.grasp_detected && !pick_) {
      world_.attached_object.clear();
      world_.attachment_stamp = {};
      world_.known_locations.erase(object_);
      world_.placement_candidates[object_] = target_id_;
      world_.release_stamps[object_] = g.stamp;
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
  std::shared_ptr<MotionTargetResolver> resolver_;
  WorldState& world_;
  bool pick_, stopping_{false};
  ManipulationPolicy policy_;
  Phase phase_{Phase::idle};
  Result result_;
  Time terminal_since_{};
  std::string object_, target_id_;
  CartesianTarget target_;
};

inline void register_demo_skills(Skills& skills, ManipulationPolicy policy = {}, VerificationPolicy verification = {}) {
  skills.define({"locate_object", "Obtain a fresh object pose", {{"object", "entity_id", "Entity to locate"}},
      "camera ready", "observation valid", "fresh object pose in a known frame"});
  skills.define({"pick_object", "Move to and grasp a previously located object", {{"object", "entity_id", "Object to grasp"}},
      "empty gripper and fresh object pose", "exclusive arm and gripper control", "grasp detected"});
  skills.define({"place_object", "Move to a located target and release the held object",
      {{"object", "entity_id", "Held object"}, {"target", "entity_id", "Release target"}},
      "requested object held and fresh target pose", "exclusive arm and gripper control", "release detected"});
  const EvidencePolicy observation_policy{policy.observation_max_age, policy.minimum_observation_quality, policy.allow_synthetic};
  const Dependencies manipulation{
      {{"motion", "arm_motion", 2}, {"gripper", "gripper", 2}, {"safety", "execution_gate", 1}},
      {"motion", "gripper"}, "safety"};
  skills.define({"verify_grasp", "Verify stable post-grasp evidence", {{"object", "entity_id", "Held object"}},
      "matching measured grasp", "read-only exclusive arm and gripper access", "accepted post-grasp evidence stable over a sample window"});
  skills.define({"verify_placement", "Verify stable post-release evidence",
      {{"object", "entity_id", "Released object"}, {"target", "entity_id", "Expected location"}},
      "matching measured release candidate", "read-only exclusive arm and gripper access", "accepted post-release evidence stable over a sample window"});
  const Dependencies verify{{{"outcome", "manipulation_observer", 1}}, {"motion", "gripper"}, ""};
  // Task aliases select one implementation ID for every step. Read-only steps
  // retain the same behavior; only pick/place gain the resolver dependency.
  for (const std::string implementation : {"standard", "pose_resolved"}) {
    const bool resolved = implementation == "pose_resolved";
    auto dependencies = manipulation;
    if (resolved) dependencies.components.push_back({"target_resolver", "motion_target_resolver", 1});
    skills.implement("locate_object", implementation, [observation_policy](Context& c) { return std::make_unique<Locate>(c, observation_policy); },
        {{{"perception", "object_locator", 2}}, {}, ""});
    skills.implement("pick_object", implementation, [policy, resolved](Context& c) { return std::make_unique<Manipulate>(c, true, policy, resolved); }, dependencies);
    skills.implement("place_object", implementation, [policy, resolved](Context& c) { return std::make_unique<Manipulate>(c, false, policy, resolved); }, dependencies);
    skills.implement("verify_grasp", implementation, [verification](Context& c) { return std::make_unique<VerifyOutcome>(c, true, verification); }, verify);
    skills.implement("verify_placement", implementation, [verification](Context& c) { return std::make_unique<VerifyOutcome>(c, false, verification); }, verify);
  }
}
}  // namespace robot_core
