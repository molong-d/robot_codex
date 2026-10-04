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

GazeboWorldAdapter::GazeboWorldAdapter(rclcpp::Node* node, Config config, robot_core::DemoScene scene)
    : node_(node), config_(std::move(config)), scene_(std::move(scene)) {
  if (config_.world_pose_topics.empty() || config_.world_frame.empty() ||
      !std::isfinite(config_.support_surface_z) || !std::isfinite(config_.object_height_m) ||
      config_.object_height_m <= 0.0 || !std::isfinite(config_.grasp_lift_m) || config_.grasp_lift_m <= 0.0 ||
      !std::isfinite(config_.placement_xy_tolerance_m) || config_.placement_xy_tolerance_m <= 0.0 ||
      !std::isfinite(config_.placement_z_tolerance_m) || config_.placement_z_tolerance_m <= 0.0 ||
      !std::isfinite(config_.stable_speed_mps) || config_.stable_speed_mps <= 0.0)
    throw std::invalid_argument("invalid Gazebo ground-truth configuration");
  for (const auto role : {robot_core::EntityRole::object, robot_core::EntityRole::target})
    for (const auto& id : scene_.ids(role)) poses_.emplace(id, PoseSample{});
  for (const auto& topic : config_.world_pose_topics) {
    if (topic.empty()) throw std::invalid_argument("empty Gazebo pose topic");
    pose_subs_.push_back(node_->create_subscription<tf2_msgs::msg::TFMessage>(topic, 10,
        [this](tf2_msgs::msg::TFMessage::ConstSharedPtr message) { on_poses(std::move(message)); }));
  }
  contact_sub_ = node_->create_subscription<robot_interfaces::msg::GraspContact>(config_.grasp_stamped_topic, 10,
      [this](robot_interfaces::msg::GraspContact::ConstSharedPtr message) {
        if (!observe_ros_stamp(node_, contact_clock_, message->source_stamp)) return;
        finger_contact_ = message->detected;
        contact_stamp_ = contact_clock_.stamp();
      });
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
          static_cast<uint64_t>(simulation_now_ns)-source_ns > 500000000ULL) continue;
      if (score > 0 && (!selected.count(entry.first) || score > selected.at(entry.first).score))
        selected[entry.first] = {score, &transform.transform, source_ns};
    }
  }
  if (selected.empty()) return;

  bool reset = false;
  for (const auto& [id, value] : selected) {
    const auto previous = source_highwater_.find(id);
    if (previous != source_highwater_.end() && value.source_ns + 100000000ULL < previous->second)
      reset = true;
  }
  if (reset) {
    ++epoch_;
    reset_reference_ = source_highwater_;
    awaiting_reset_epoch_.clear();
    for (const auto& [id, stamp] : reset_reference_) {
      (void)stamp;
      awaiting_reset_epoch_.insert(id);
    }
    source_highwater_.clear();
    for (auto& entry : poses_) entry.second = PoseSample{};
  }

  const auto receipt = robot_core::Clock::now();
  for (const auto& entry : selected) {
    const auto id = entry.first;
    const auto& selected_pose = entry.second;
    const auto waiting = awaiting_reset_epoch_.find(id);
    if (waiting != awaiting_reset_epoch_.end()) {
      const auto old = reset_reference_.find(id);
      if (old == reset_reference_.end() || selected_pose.source_ns + 100000000ULL >= old->second) continue;
      awaiting_reset_epoch_.erase(waiting);
    }
    const auto old_source = source_highwater_.find(id);
    if (old_source != source_highwater_.end() && selected_pose.source_ns <= old_source->second) continue;
    const auto pose = as_pose(*selected_pose.transform);
    if (!robot_core::valid_pose(pose)) continue;
    auto& sample = poses_.at(id);
    sample.previous_pose = sample.pose;
    sample.previous_stamp = sample.stamp;
    sample.pose = pose;
    sample.stamp = receipt;
    sample.sample_id = ++world_sample_sequence_;
    sample.source_ns = selected_pose.source_ns;
    sample.valid = true;
    source_highwater_[id] = selected_pose.source_ns;
  }
}

const GazeboWorldAdapter::PoseSample* GazeboWorldAdapter::fresh_pose(const std::string& id,
                                                                     robot_core::Time now) const {
  const auto found = poses_.find(id);
  if (found == poses_.end() || !found->second.valid ||
      !robot_core::fresh(found->second.stamp, now, std::chrono::milliseconds(500))) return nullptr;
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
  if (!sample) return std::nullopt;
  const bool contact = finger_contact_ && robot_core::fresh(contact_stamp_, now, std::chrono::milliseconds(500));
  const bool lifted = sample->pose.z >= config_.support_surface_z+config_.object_height_m/2.0+config_.grasp_lift_m;
  return robot_core::OutcomeEvidence{object, "", sample->stamp, true, contact && lifted,
      {evidence_source(), true, 1.0}, sample->sample_id};
}

std::optional<robot_core::OutcomeEvidence> GazeboWorldAdapter::placement(const std::string& object,
    const std::string& target, robot_core::Time now) {
  const auto* sample = fresh_pose(object, now);
  const auto* goal = fresh_pose(target, now);
  if (!sample || !goal) return std::nullopt;
  const bool contact = finger_contact_ && robot_core::fresh(contact_stamp_, now, std::chrono::milliseconds(500));
  const double speed = sample->previous_stamp < sample->stamp ?
      distance(sample->pose, sample->previous_pose)/
          std::chrono::duration<double>(sample->stamp-sample->previous_stamp).count() :
      std::numeric_limits<double>::infinity();
  const bool in_region = std::abs(sample->pose.x-goal->pose.x) <= config_.placement_xy_tolerance_m &&
                         std::abs(sample->pose.y-goal->pose.y) <= config_.placement_xy_tolerance_m &&
                         std::abs(sample->pose.z-(config_.support_surface_z+config_.object_height_m/2.0)) <=
                             config_.placement_z_tolerance_m;
  const bool condition = !contact && in_region && speed <= config_.stable_speed_mps;
  return robot_core::OutcomeEvidence{object, target, sample->stamp, true, condition,
      {evidence_source(), true, 1.0}, sample->sample_id};
}
}  // namespace robot_ros_adapters
