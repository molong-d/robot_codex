#include "robot_ros_adapters/adapters.hpp"
// Compiling the public adapter header here also validates it independently of runtime.

#include <cmath>

namespace robot_ros_adapters {
namespace {
double distance(const robot_core::Pose& a, const robot_core::Pose& b) {
  const auto dx = a.x-b.x, dy = a.y-b.y, dz = a.z-b.z;
  return std::sqrt(dx*dx+dy*dy+dz*dz);
}
int frame_score(const std::string& frame, const std::string& id) {
  if (frame == id) return 3;
  if (frame.size() >= id.size()+2 && frame.compare(frame.size()-id.size(), id.size(), id) == 0 &&
      frame.compare(frame.size()-id.size()-2, 2, "::") == 0) return 2;
  return frame.find(id+"::") != std::string::npos ? 1 : 0;
}
robot_core::Pose as_pose(const geometry_msgs::msg::Transform& transform) {
  return {transform.translation.x, transform.translation.y, transform.translation.z,
          transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w};
}
}  // namespace

GazeboWorldAdapter::GazeboWorldAdapter(rclcpp::Node* node, Config config, robot_core::DemoScene scene,
                                       robot_core::WorldState* world_state)
    : node_(node), config_(std::move(config)), scene_(std::move(scene)), world_state_(world_state) {
  if (config_.world_pose_topics.empty() || config_.world_frame.empty() ||
      !std::isfinite(config_.support_surface_z) || !std::isfinite(config_.object_height_m) ||
      config_.object_height_m <= 0.0 || !std::isfinite(config_.grasp_lift_m) || config_.grasp_lift_m <= 0.0 ||
      !std::isfinite(config_.placement_xy_tolerance_m) || config_.placement_xy_tolerance_m <= 0.0 ||
      !std::isfinite(config_.placement_z_tolerance_m) || config_.placement_z_tolerance_m <= 0.0 ||
      !std::isfinite(config_.stable_speed_mps) || config_.stable_speed_mps <= 0.0 ||
      config_.evidence_max_age.count() <= 0 || config_.contact_pose_pairing_tolerance.count() <= 0)
    throw std::invalid_argument("invalid Gazebo ground-truth configuration");
  for (const auto role : {robot_core::EntityRole::object, robot_core::EntityRole::target})
    for (const auto& id : scene_.ids(role)) {
      poses_.emplace(id, PoseSample{});
      pose_clocks_.emplace(id, MonotonicSampleClock(config_.evidence_max_age));
    }
  for (const auto& topic : config_.world_pose_topics) {
    if (topic.empty()) throw std::invalid_argument("empty Gazebo pose topic");
    pose_subs_.push_back(node_->create_subscription<tf2_msgs::msg::TFMessage>(topic, 10,
        [this](tf2_msgs::msg::TFMessage::ConstSharedPtr message) { on_poses(std::move(message)); }));
  }
  contact_clock_ = MonotonicSampleClock(config_.evidence_max_age);
  contact_sub_ = node_->create_subscription<robot_interfaces::msg::GraspContact>(config_.grasp_stamped_topic, 10,
      [this](robot_interfaces::msg::GraspContact::ConstSharedPtr message) {
        const auto old_epoch = contact_clock_.epoch();
        const bool accepted = observe_ros_stamp(node_, contact_clock_, message->source_stamp);
        if (contact_clock_.epoch() != old_epoch) {
          activate_epoch(contact_clock_.epoch());
          finger_contact_ = rc::ContactState::unknown;
          contact_stamp_ = {};
          contact_source_ns_ = 0;
        }
        if (!accepted || message->schema_version != 2 || message->frame_id != config_.world_frame || message->object_id != "workpiece" ||
            message->epoch != contact_clock_.epoch() || message->sequence == 0 ||
            message->sequence <= contact_sequence_) return;
        switch (message->state) {
          case robot_interfaces::msg::GraspContact::NO_FINGER_CONTACT: finger_contact_ = rc::ContactState::none; break;
          case robot_interfaces::msg::GraspContact::LEFT_FINGER_ONLY: finger_contact_ = rc::ContactState::left_only; break;
          case robot_interfaces::msg::GraspContact::RIGHT_FINGER_ONLY: finger_contact_ = rc::ContactState::right_only; break;
          case robot_interfaces::msg::GraspContact::BOTH_FINGERS: finger_contact_ = rc::ContactState::both; break;
          default: finger_contact_ = rc::ContactState::unknown; break;
        }
        contact_stamp_ = contact_clock_.stamp();
        contact_source_ns_ = contact_clock_.source_ns();
        contact_epoch_ = contact_clock_.epoch();
        contact_sequence_ = message->sequence;
      });
}

GazeboPlanningSceneGate::GazeboPlanningSceneGate(rclcpp::Node* node, const std::string& topic) {
  if (!node || topic.empty()) throw std::invalid_argument("planning scene status requires node and topic");
  sub_ = node->create_subscription<robot_interfaces::msg::PlanningSceneStatus>(topic, 10,
      [this](robot_interfaces::msg::PlanningSceneStatus::ConstSharedPtr message) {
        if (!message || message->schema_version != 1 || message->producer_id.empty() ||
            message->sequence == 0) return;
        if (state_.valid && message->producer_id != producer_id_) {
          if (message->ready) return;  // a producer restart must first publish unsynchronized state
          state_ = {};
          producer_id_ = message->producer_id;
          last_sim_time_ns_ = 0;
          rewind_pending_ = false;
        }
        if (message->sequence <= state_.sequence) return;
        if (state_.valid && message->epoch < state_.epoch) return;
        if (!state_.valid || message->producer_id != producer_id_) {
          state_ = {};
          producer_id_ = message->producer_id;
        }
        const uint64_t sim_ns = static_cast<uint64_t>(message->source_clock_stamp.sec) * 1000000000ULL +
                                message->source_clock_stamp.nanosec;
        if (last_sim_time_ns_ != 0 && sim_ns != 0 && sim_ns < last_sim_time_ns_) {
          rewind_pending_ = true;
          previous_epoch_ = state_.epoch;
        }
        last_sim_time_ns_ = sim_ns;
        state_.valid = true;
        if (rewind_pending_ && message->epoch > previous_epoch_) rewind_pending_ = false;
        state_.ready = message->ready && !rewind_pending_;
        state_.request_in_flight = message->request_in_flight;
        state_.cleanup_pending = message->cleanup_pending;
        state_.epoch = message->epoch;
        state_.sequence = message->sequence;
        state_.desired_version = message->desired_version;
        state_.confirmed_version = message->confirmed_version;
        state_.desired_attached_object = message->desired_attached_object;
        state_.confirmed_attached_object = message->confirmed_attached_object;
        state_.received_at = rc::Clock::now();
      });
}

void GazeboWorldAdapter::activate_epoch(uint64_t epoch) {
  if (epoch <= epoch_) return;
  epoch_ = epoch;
  for (auto& entry : poses_) entry.second = PoseSample{};
  finger_contact_ = rc::ContactState::unknown;
  contact_stamp_ = {};
  contact_source_ns_ = 0;
  contact_epoch_ = epoch;
  contact_sequence_ = 0;
  if (world_state_) {
    world_state_->observations.clear();
    world_state_->attached_object.clear();
    world_state_->known_locations.clear();
    world_state_->placement_candidates.clear();
    world_state_->attachment_stamp = {};
    world_state_->attachment_source_time_ns = 0;
    world_state_->attachment_epoch = epoch;
    world_state_->release_stamps.clear();
    world_state_->release_source_times_ns.clear();
    world_state_->release_epochs.clear();
    world_state_->grasp_verifications.clear();
    world_state_->placement_verifications.clear();
  }
}

void GazeboWorldAdapter::on_poses(tf2_msgs::msg::TFMessage::ConstSharedPtr message) {
  if (!message || message->transforms.empty()) return;
  struct Selected { int score; const geometry_msgs::msg::Transform* transform; uint64_t source_ns; };
  std::map<std::string, Selected> selected;
  const auto simulation_now_ns = node_->now().nanoseconds();
  if (simulation_now_ns <= 0) return;
  for (const auto& entry : poses_) {
    for (const auto& transform : message->transforms) {
      if (transform.header.frame_id != config_.world_frame &&
          transform.header.frame_id != config_.gazebo_world_name) continue;
      const int score = frame_score(transform.child_frame_id, entry.first);
      const uint64_t source_ns = static_cast<uint64_t>(transform.header.stamp.sec) * 1000000000ULL +
                                 transform.header.stamp.nanosec;
      if (source_ns == 0 || source_ns > static_cast<uint64_t>(simulation_now_ns) ||
          static_cast<uint64_t>(simulation_now_ns)-source_ns >
              static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(config_.evidence_max_age).count())) continue;
      if (score > 0 && (!selected.count(entry.first) || score > selected.at(entry.first).score))
        selected[entry.first] = {score, &transform.transform, source_ns};
    }
  }
  if (selected.empty()) return;

  for (const auto& entry : selected) {
    const auto id = entry.first;
    const auto& selected_pose = entry.second;
    auto& clock = pose_clocks_.at(id);
    const auto old_epoch = clock.epoch();
    if (!clock.observe(static_cast<int64_t>(selected_pose.source_ns), simulation_now_ns, robot_core::Clock::now())) {
      if (clock.epoch() != old_epoch) activate_epoch(clock.epoch());
      continue;
    }
    if (clock.epoch() != old_epoch) activate_epoch(clock.epoch());
    if (clock.epoch() != epoch_) continue;
    const auto pose = as_pose(*selected_pose.transform);
    if (!robot_core::valid_pose(pose)) continue;
    auto& sample = poses_.at(id);
    sample.previous_pose = sample.pose;
    sample.previous_stamp = sample.stamp;
    sample.previous_source_ns = sample.source_ns;
    sample.pose = pose;
    sample.stamp = clock.stamp();
    // The configured Gazebo world name was explicitly accepted above as an
    // alias for the canonical world frame.
    sample.frame_id = config_.world_frame;
    sample.sample_id = ++world_sample_sequence_;
    sample.source_ns = static_cast<uint64_t>(clock.source_ns());
    sample.epoch = clock.epoch();
    sample.valid = true;
  }
}

const GazeboWorldAdapter::PoseSample* GazeboWorldAdapter::fresh_pose(const std::string& id,
                                                                     robot_core::Time now) const {
  const auto found = poses_.find(id);
  if (found == poses_.end() || !found->second.valid ||
      found->second.epoch != epoch_ || found->second.frame_id != config_.world_frame ||
      !robot_core::fresh(found->second.stamp, now, config_.evidence_max_age)) return nullptr;
  return &found->second;
}

std::string GazeboWorldAdapter::evidence_source() const {
  return "gazebo_truth_epoch_"+std::to_string(epoch_);
}

std::optional<robot_core::Observation> GazeboWorldAdapter::locate(const std::string& id, robot_core::Time now) {
  try { (void)scene_.at(id); } catch (const std::out_of_range&) { return std::nullopt; }
  const auto* sample = fresh_pose(id, now);
  if (!sample) {
    const auto found = poses_.find(id);
    const bool received = found != poses_.end() && found->second.valid;
    const double age_ms = received ? std::chrono::duration<double, std::milli>(now-found->second.stamp).count() : -1.0;
    RCLCPP_WARN(node_->get_logger(),
        "Gazebo truth pose unavailable for %s (received=%s, age_ms=%.1f, expected_parent=%s or %s)",
        id.c_str(), received ? "true" : "false", age_ms, config_.world_frame.c_str(), config_.gazebo_world_name.c_str());
    return std::nullopt;
  }
  return robot_core::Observation{id, config_.world_frame, sample->pose, sample->stamp, true,
      {evidence_source(), true, 1.0}, robot_core::PoseMeaning::object_pose};
}

std::optional<robot_core::OutcomeEvidence> GazeboWorldAdapter::grasp(const std::string& object,
                                                                     robot_core::Time now) {
  const auto* sample = fresh_pose(object, now);
  const bool contact_fresh = contact_sequence_ != 0 && contact_epoch_ == epoch_ &&
      contact_clock_.epoch() == epoch_ && robot_core::fresh(contact_stamp_, now, config_.evidence_max_age);
  if (!sample || !contact_fresh) return std::nullopt;
  const bool contact = rc::dual_finger_grasp(finger_contact_);
  const bool lifted = sample->pose.z >= config_.support_surface_z+config_.object_height_m/2.0+config_.grasp_lift_m;
  robot_core::OutcomeEvidence evidence{object, "", sample->stamp, true, contact && lifted,
      {evidence_source(), true, 1.0}, sample->sample_id, sample->source_ns, sample->epoch};
  evidence.contact_state = finger_contact_;
  evidence.contact_source_time_ns = contact_source_ns_;
  return evidence;
}

std::optional<robot_core::OutcomeEvidence> GazeboWorldAdapter::placement(const std::string& object,
    const std::string& target, robot_core::Time now) {
  const auto* sample = fresh_pose(object, now);
  const auto* goal = fresh_pose(target, now);
  const bool contact_fresh = contact_sequence_ != 0 && contact_epoch_ == epoch_ &&
      contact_clock_.epoch() == epoch_ && robot_core::fresh(contact_stamp_, now, config_.evidence_max_age);
  if (!sample || !goal || !contact_fresh || sample->epoch != goal->epoch ||
      contact_source_ns_ == 0 || contact_epoch_ != sample->epoch) return std::nullopt;
  const auto pairing_limit_ns = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
      config_.contact_pose_pairing_tolerance).count());
  const auto absolute_delta = [](uint64_t left, uint64_t right) {
    return left > right ? left - right : right - left;
  };
  if (absolute_delta(sample->source_ns, goal->source_ns) > pairing_limit_ns ||
      absolute_delta(sample->source_ns, contact_source_ns_) > pairing_limit_ns) return std::nullopt;
  bool has_release = false;
  uint64_t release_source_ns = 0;
  if (world_state_) {
    const auto release_time = world_state_->release_source_times_ns.find(object);
    const auto release_epoch = world_state_->release_epochs.find(object);
    const auto release_receipt = world_state_->release_stamps.find(object);
    has_release = release_time != world_state_->release_source_times_ns.end() &&
        release_epoch != world_state_->release_epochs.end() && release_receipt != world_state_->release_stamps.end() &&
        release_time->second != 0 && release_receipt->second != rc::Time{} &&
        release_epoch->second == epoch_ && contact_epoch_ == epoch_ &&
        contact_source_ns_ > release_time->second && sample->source_ns > release_time->second;
    if (has_release) release_source_ns = release_time->second;
  }
  const bool confirmed_no_contact = has_release && rc::no_finger_contact(finger_contact_);
  const double speed = sample->previous_source_ns != 0 && sample->source_ns > sample->previous_source_ns &&
      sample->epoch == epoch_ ?
      distance(sample->pose, sample->previous_pose)/
          (static_cast<double>(sample->source_ns-sample->previous_source_ns)*1e-9) :
      std::numeric_limits<double>::infinity();
  const double xy_error = std::hypot(sample->pose.x-goal->pose.x, sample->pose.y-goal->pose.y);
  const bool in_region = xy_error <= config_.placement_xy_tolerance_m &&
                         std::abs(sample->pose.z-(config_.support_surface_z+config_.object_height_m/2.0)) <=
                             config_.placement_z_tolerance_m;
  const bool condition = confirmed_no_contact && sample->source_ns > release_source_ns &&
                         contact_source_ns_ > release_source_ns && in_region && speed <= config_.stable_speed_mps;
  robot_core::OutcomeEvidence evidence{object, target, sample->stamp, true, condition,
      {evidence_source(), true, 1.0}, sample->sample_id, sample->source_ns, sample->epoch};
  evidence.contact_state = (rc::no_finger_contact(finger_contact_) && !has_release) ?
      rc::ContactState::unknown : finger_contact_;
  evidence.contact_source_time_ns = contact_source_ns_;
  return evidence;
}
}  // namespace robot_ros_adapters
