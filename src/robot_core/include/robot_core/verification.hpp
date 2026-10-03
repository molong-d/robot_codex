#pragma once
#include "robot_core/components.hpp"

namespace robot_core {
// Goal semantics, temporal acceptance and world-state promotion belong to the
// skill. The observer only reports technical evidence and its provenance.
class VerifyOutcome final : public Skill {
 public:
  VerifyOutcome(Context& context, bool grasp, VerificationPolicy policy = {})
      : observer_(context.bindings.get<ManipulationObserver>("outcome")), world_(context.world),
        grasp_(grasp), policy_(std::move(policy)) {
    if (!valid_evidence_policy(policy_.evidence) || policy_.stable_duration.count() <= 0 ||
        policy_.minimum_samples < 2 || policy_.minimum_samples > 1000)
      throw std::invalid_argument("invalid verification policy");
  }
  Result start(const Arguments& args, Time now) override {
    object_ = args.at("object");
    if (grasp_) {
      world_.grasp_verifications.erase(object_);
      if (world_.attached_object != object_ || world_.attachment_stamp == Time{})
        return done(Status::failed, "NOT_HOLDING", "matching measured grasp required before verification");
      effect_stamp_ = world_.attachment_stamp;
    } else {
      target_ = args.at("target");
      world_.placement_verifications.erase(object_);
      const auto candidate = world_.placement_candidates.find(object_);
      const auto release = world_.release_stamps.find(object_);
      if (world_.attached_object == object_ || candidate == world_.placement_candidates.end() ||
          candidate->second != target_ || release == world_.release_stamps.end() || release->second == Time{})
        return done(Status::failed, "NO_RELEASE_CANDIDATE", "matching measured release required before verification");
      effect_stamp_ = release->second;
    }
    return sample(now);
  }
  Result tick(Time now) override {
    if (terminal(result_.status)) return result_;
    if (cancel_requested_) return done(Status::canceled, "CANCELED", "read-only verification stopped");
    return sample(now);
  }
  void cancel() override { cancel_requested_ = true; }
 private:
  Result sample(Time now) {
    const auto value = grasp_ ? observer_->grasp(object_, now) : observer_->placement(object_, target_, now);
    if (!value || !value->valid || value->sample_id == 0 || value->object_id != object_ || value->target_id != target_ ||
        !acceptable_evidence(value->evidence, value->stamp, now, policy_.evidence))
      return done(Status::failed, "INVALID_VERIFICATION_EVIDENCE", "fresh identified evidence with accepted source/quality required");
    if (value->stamp <= effect_stamp_)
      return result_ = {Status::running, "PRE_EFFECT_EVIDENCE", "awaiting evidence captured after grasp/release"};
    if (!value->condition_met)
      return done(Status::failed, grasp_ ? "GRASP_NOT_VERIFIED" : "PLACEMENT_NOT_VERIFIED", "observer reports condition not met");
    if (samples_ != 0 && (value->evidence.source != source_ || value->evidence.synthetic != synthetic_))
      return done(Status::failed, "VERIFICATION_SOURCE_CHANGED", "cannot combine samples from different sources");
    if (samples_ != 0 && (value->stamp < last_stamp_ || value->sample_id < last_sample_))
      return done(Status::failed, "OUT_OF_ORDER_EVIDENCE", "verification evidence went backwards");
    if (samples_ != 0 && value->stamp-last_stamp_ > policy_.evidence.max_age)
      return done(Status::failed, "EVIDENCE_GAP", "verification sample gap exceeds evidence age limit");
    if (value->stamp != last_stamp_ && value->sample_id != last_sample_) {
      if (samples_ == 0) { first_stamp_ = value->stamp; source_ = value->evidence.source; synthetic_ = value->evidence.synthetic; }
      ++samples_; last_stamp_ = value->stamp; last_sample_ = value->sample_id;
    }
    const auto span = std::chrono::duration_cast<std::chrono::milliseconds>(last_stamp_-first_stamp_);
    if (samples_ < policy_.minimum_samples || span < policy_.stable_duration)
      return result_ = {Status::running, "", "collecting distinct post-effect verification samples"};
    const OutcomeVerification verified{*value, samples_, span};
    if (grasp_) world_.grasp_verifications[object_] = verified;
    else {
      world_.placement_verifications[object_] = verified;
      world_.known_locations[object_] = target_;
      world_.placement_candidates.erase(object_);
    }
    return done(Status::succeeded, "", grasp_ ? "grasp evidence stable over verification window" : "placement evidence stable over verification window");
  }
  Result done(Status status, std::string code, std::string message) {
    return result_ = {status, std::move(code), std::move(message)};
  }
  std::shared_ptr<ManipulationObserver> observer_;
  WorldState& world_;
  bool grasp_, cancel_requested_{false}, synthetic_{false};
  VerificationPolicy policy_;
  std::string object_, target_, source_;
  Time effect_stamp_{}, first_stamp_{}, last_stamp_{};
  unsigned samples_{0};
  uint64_t last_sample_{0};
  Result result_;
};
}  // namespace robot_core
