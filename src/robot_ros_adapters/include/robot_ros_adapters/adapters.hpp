#pragma once
#include "robot_core/components.hpp"
#include "robot_core/task_catalog.hpp"
#include "robot_ros_adapters/joint_position.hpp"
#include "robot_ros_adapters/time_source.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit_msgs/action/move_group.hpp>
#include <robot_interfaces/msg/grasp_contact.hpp>
#include <control_msgs/action/parallel_gripper_command.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_msgs/msg/tf_message.hpp>
#include <functional>
#include <set>

namespace robot_ros_adapters {
namespace rc = robot_core;
struct Config {
  std::string move_action{"/move_action"};
  std::string gripper_action{"/panda_hand_controller/gripper_cmd"};
  std::string group{"panda_arm"}, frame{"panda_link0"}, tip{"panda_hand"};
  std::vector<std::string> arm_joints{"panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"};
  std::string finger_joint{"panda_finger_joint1"};
  std::string joint_states_topic{"/joint_states"}, grasp_topic{"/grasp_detected"};
  std::vector<std::string> world_pose_topics{"/panda_gz/workpiece/pose", "/panda_gz/tray/pose"};
  std::string grasp_stamped_topic{"/panda/grasp_contact_stamped"};
  std::string world_frame{"world"};
  std::string gazebo_world_name{"panda_pick_place"};
  std::string arm_resource{"panda_arm"}, gripper_resource{"panda_gripper"};
  bool simulation_grasp_detection{false};
  double gripper_release_width_m{0.08};
  double support_surface_z{0.35}, object_height_m{0.04}, grasp_lift_m{0.04};
  double placement_xy_tolerance_m{0.05}, placement_z_tolerance_m{0.015}, stable_speed_mps{0.04};
  std::chrono::milliseconds evidence_max_age{500}, contact_pose_pairing_tolerance{100};
  double approach_clearance_m{0.0}, post_action_clearance_m{0.0};
  double velocity_scaling{0.2}, acceleration_scaling{0.2};
};

inline bool observe_ros_stamp(rclcpp::Node* node, MonotonicSampleClock& tracker,
                              const builtin_interfaces::msg::Time& stamp) {
  return tracker.observe(rclcpp::Time(stamp, node->get_clock()->get_clock_type()).nanoseconds(),
                         node->now().nanoseconds(), rc::Clock::now());
}

// One executor owns the channel and all its callbacks. No blocking spin/wait,
// no command replay, and a pending acceptance cannot escape an earlier cancel.
template<class Action> class ActionChannel {
 public:
  using Handle = rclcpp_action::ClientGoalHandle<Action>;
  using Wrapped = typename Handle::WrappedResult;
  ActionChannel(rclcpp::Node* node, const std::string& name)
      : client_(rclcpp_action::create_client<Action>(node, name)) {}
  void send(const typename Action::Goal& goal, std::function<bool(const Wrapped&)> success) {
    if (!returned_) throw std::runtime_error("action already active");
    if (!client_->action_server_is_ready()) throw std::runtime_error("action server unavailable");
    stopped_ = false; returned_ = false; handle_.reset(); status_ = rc::Status::running;
    typename rclcpp_action::Client<Action>::SendGoalOptions options;
    options.goal_response_callback = [this](typename Handle::SharedPtr handle) {
      handle_ = handle;
      if (!handle) { returned_ = true; status_ = rc::Status::failed; return; }
      if (stopped_) client_->async_cancel_goal(handle_);
    };
    options.result_callback = [this, success](const Wrapped& result) {
      returned_ = true;
      if (result.code == rclcpp_action::ResultCode::CANCELED) status_ = rc::Status::canceled;
      else if (result.code == rclcpp_action::ResultCode::SUCCEEDED && result.result && success(result)) status_ = rc::Status::succeeded;
      else if (result.code == rclcpp_action::ResultCode::UNKNOWN) status_ = rc::Status::faulted;
      else status_ = rc::Status::failed;
    };
    client_->async_send_goal(goal, options);
  }
  void stop() {
    if (stopped_ || returned_) return;
    stopped_ = true;
    if (handle_) client_->async_cancel_goal(handle_);
  }
  rc::Status status() const { return !returned_ && stopped_ ? rc::Status::canceling : status_; }
 private:
  typename rclcpp_action::Client<Action>::SharedPtr client_;
  typename Handle::SharedPtr handle_;
  bool returned_{true}, stopped_{false};
  rc::Status status_{rc::Status::idle};
};

inline bool stationary(const sensor_msgs::msg::JointState& s, const std::vector<std::string>& joints) {
  for (const auto& joint : joints) {
    auto it = std::find(s.name.begin(), s.name.end(), joint);
    if (it == s.name.end()) return false;
    const auto i = static_cast<size_t>(it-s.name.begin());
    if (i >= s.velocity.size() || !std::isfinite(s.velocity[i]) || std::abs(s.velocity[i]) > 0.001) return false;
  }
  return true;
}

class MoveItArm final : public rc::ArmMotion {
 public:
  MoveItArm(rclcpp::Node* node, Config config)
      : node_(node), config_(std::move(config)), channel_(node, config_.move_action),
        buffer_(std::make_shared<tf2_ros::Buffer>(node->get_clock())),
    listener_(std::make_shared<tf2_ros::TransformListener>(*buffer_, node, false)),
    joint_clock_(config_.evidence_max_age), tf_clock_(config_.evidence_max_age) {
    sub_ = node->create_subscription<sensor_msgs::msg::JointState>(config_.joint_states_topic, 10,
        [this](sensor_msgs::msg::JointState::ConstSharedPtr s) {
          const auto old_epoch = joint_clock_.epoch();
          if (observe_ros_stamp(node_, joint_clock_, s->header.stamp)) joints_ = *s;
          if (joint_clock_.epoch() != old_epoch) joints_ = sensor_msgs::msg::JointState{};
        });
    if (config_.arm_joints.empty() || config_.arm_resource.empty() ||
        config_.velocity_scaling <= 0.0 || config_.velocity_scaling > 1.0 ||
        config_.acceleration_scaling <= 0.0 || config_.acceleration_scaling > 1.0)
      throw std::invalid_argument("invalid MoveIt adapter configuration");
  }
  std::string resource_id() const override { return config_.arm_resource; }
  void begin_move(const rc::CartesianTarget& target) override {
    if (target.frame_id != config_.frame || !rc::valid_pose(target.pose) || !rc::valid_tolerance(target.tolerance))
      throw std::invalid_argument("invalid MoveIt target");
    moveit_msgs::action::MoveGroup::Goal goal;
    goal.request.group_name = config_.group;
    goal.request.num_planning_attempts = 1;
    goal.request.allowed_planning_time = 5.0;
    goal.request.max_velocity_scaling_factor = config_.velocity_scaling;
    goal.request.max_acceleration_scaling_factor = config_.acceleration_scaling;
    goal.request.start_state.is_diff = true;
    goal.planning_options.planning_scene_diff.is_diff = true;
    goal.planning_options.planning_scene_diff.robot_state.is_diff = true;
    moveit_msgs::msg::Constraints constraints;
    moveit_msgs::msg::PositionConstraint position;
    position.header.frame_id = config_.frame; position.link_name = config_.tip; position.weight = 1.0;
    shape_msgs::msg::SolidPrimitive sphere;
    sphere.type = shape_msgs::msg::SolidPrimitive::SPHERE;
    // Plan inside the acceptance envelope, leaving margin for numerical error.
    sphere.dimensions = {target.tolerance.position_m/2.0};
    geometry_msgs::msg::Pose center;
    center.position.x = target.pose.x; center.position.y = target.pose.y; center.position.z = target.pose.z;
    center.orientation.w = 1.0;
    position.constraint_region.primitives.push_back(sphere);
    position.constraint_region.primitive_poses.push_back(center);
    moveit_msgs::msg::OrientationConstraint orientation;
    orientation.header.frame_id = config_.frame; orientation.link_name = config_.tip;
    orientation.orientation.x = target.pose.qx; orientation.orientation.y = target.pose.qy;
    orientation.orientation.z = target.pose.qz; orientation.orientation.w = target.pose.qw;
    // Per-axis bounds do not equal the total quaternion angular error. With a
    // rotation-vector parameterization, norm <= sqrt(3) * axis_bound.
    orientation.parameterization = moveit_msgs::msg::OrientationConstraint::ROTATION_VECTOR;
    const double axis_bound = target.tolerance.orientation_rad/(2.0*std::sqrt(3.0));
    orientation.absolute_x_axis_tolerance = axis_bound;
    orientation.absolute_y_axis_tolerance = axis_bound;
    orientation.absolute_z_axis_tolerance = axis_bound;
    orientation.weight = 1.0;
    constraints.position_constraints.push_back(position);
    constraints.orientation_constraints.push_back(orientation);
    goal.request.goal_constraints.push_back(constraints);
    channel_.send(goal, [](const auto& r) { return r.result->error_code.val == 1; });
  }
  rc::Status poll(rc::Time now) override {
    measured_ = {};
    try {
      const auto t = buffer_->lookupTransform(config_.frame, config_.tip, tf2::TimePointZero);
      const auto old_epoch = tf_clock_.epoch();
      if (!observe_ros_stamp(node_, tf_clock_, t.header.stamp)) return channel_.status();
      if (tf_clock_.epoch() != old_epoch) measured_ = {};
      measured_.frame_id = config_.frame;
      measured_.pose = {t.transform.translation.x, t.transform.translation.y, t.transform.translation.z,
                        t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w};
      measured_.stamp = std::min(joint_clock_.stamp(), tf_clock_.stamp());
      measured_.sample_id = std::min(joint_clock_.sample_id(), tf_clock_.sample_id());
      measured_.epoch = tf_clock_.epoch();
      measured_.valid = joint_clock_.epoch() == tf_clock_.epoch() && measured_.stamp != rc::Time{} &&
                        rc::fresh(measured_.stamp, now, config_.evidence_max_age) && rc::valid_pose(measured_.pose);
      measured_.stopped = measured_.valid && stationary(joints_, config_.arm_joints);
    } catch (const tf2::TransformException&) { /* missing TF is unavailable feedback */ }
    return channel_.status();
  }
  void request_stop() override { channel_.stop(); }
  rc::MotionFeedback feedback() const override { return measured_; }
 private:
  rclcpp::Node* node_;
  Config config_;
  ActionChannel<moveit_msgs::action::MoveGroup> channel_;
  std::shared_ptr<tf2_ros::Buffer> buffer_;
  std::shared_ptr<tf2_ros::TransformListener> listener_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
  sensor_msgs::msg::JointState joints_;
  rc::MotionFeedback measured_;
  MonotonicSampleClock joint_clock_, tf_clock_;
};

class ParallelGripper final : public rc::Gripper {
 public:
  ParallelGripper(rclcpp::Node* node, Config config)
      : node_(node), config_(std::move(config)), channel_(node, config_.gripper_action),
        joint_clock_(config_.evidence_max_age), contact_clock_(config_.evidence_max_age) {
    sub_ = node->create_subscription<sensor_msgs::msg::JointState>(config_.joint_states_topic, 10,
        [this](sensor_msgs::msg::JointState::ConstSharedPtr s) {
          const auto old_epoch = joint_clock_.epoch();
          if (observe_ros_stamp(node_, joint_clock_, s->header.stamp)) joints_ = *s;
          if (joint_clock_.epoch() != old_epoch) joints_ = sensor_msgs::msg::JointState{};
        });
    if (config_.grasp_stamped_topic.empty()) {
      contact_ = node->create_subscription<std_msgs::msg::Bool>(config_.grasp_topic, 10,
          [this](std_msgs::msg::Bool::ConstSharedPtr m) {
          contact_value_ = m->data;
          contact_stamp_ = rc::Clock::now();
          contact_source_ns_ = 0;
          contact_epoch_ = 0;
        });
    } else {
      stamped_contact_ = node->create_subscription<robot_interfaces::msg::GraspContact>(
          config_.grasp_stamped_topic, 10, [this](robot_interfaces::msg::GraspContact::ConstSharedPtr m) {
            const auto old_epoch = contact_clock_.epoch();
            const bool accepted = observe_ros_stamp(node_, contact_clock_, m->source_stamp);
            if (contact_clock_.epoch() != old_epoch) {
              contact_value_ = false;
              contact_stamp_ = {};
              contact_source_ns_ = 0;
            }
            if (!accepted || m->frame_id != config_.world_frame || m->object_id != "workpiece" ||
                m->epoch != contact_clock_.epoch() || m->sequence == 0 ||
                m->sequence <= contact_message_sequence_) return;
            contact_value_ = m->detected;
            contact_stamp_ = contact_clock_.stamp();
            contact_source_ns_ = contact_clock_.source_ns();
            contact_epoch_ = contact_clock_.epoch();
            contact_message_sequence_ = m->sequence;
          });
    }
  }
  std::string resource_id() const override { return config_.gripper_resource; }
  void begin_grasp(const rc::GraspCommand& command) override {
    closing_ = true; commanded_width_ = command.width_m;
    send(command.width_m, command.max_effort_n);
  }
  void begin_release(double width_m) override {
    closing_ = false; commanded_width_ = width_m; send(width_m, 20.0);
  }
  rc::Status poll(rc::Time now) override {
    measured_ = {};
    bool joint_fresh = false;
    bool contact_fresh = false;
    auto it = std::find(joints_.name.begin(), joints_.name.end(), config_.finger_joint);
    if (it != joints_.name.end()) {
        const auto i = static_cast<size_t>(it-joints_.name.begin());
      if (i < joints_.position.size()) {
        double first_position = 0.0;
        const bool first_position_valid = panda_joint_position(joints_.position[i], first_position);
        measured_.width_m = 2.0*first_position;  // Panda's single actuated finger has a symmetric mate.
        measured_.stamp = joint_clock_.stamp();
        measured_.sample_id = joint_clock_.sample_id();
        measured_.valid = measured_.stamp != rc::Time{} &&
                          rc::fresh(measured_.stamp, now, config_.evidence_max_age) &&
                          first_position_valid && std::isfinite(measured_.width_m) && measured_.width_m >= 0.0;
        joint_fresh = measured_.valid;
        measured_.stopped = measured_.valid && stationary(joints_, {config_.finger_joint});
      }
    }
    const auto action_status = channel_.status();
    if (config_.simulation_grasp_detection) {
      // Explicit synthetic contact for GenericSystem demo only. No contact physics.
      if (measured_.valid && measured_.stopped && action_status != rc::Status::idle &&
          std::abs(measured_.width_m-commanded_width_) <= 0.002) sim_holding_ = closing_;
      measured_.grasp_detected = sim_holding_;
    } else {
      contact_fresh = rc::fresh(contact_stamp_, now, config_.evidence_max_age);
      contact_fresh = contact_fresh && contact_epoch_ == contact_clock_.epoch() &&
                      contact_clock_.epoch() == joint_clock_.epoch();
      measured_.valid = measured_.valid && contact_fresh;
      measured_.grasp_detected = contact_value_;
      measured_.stamp = std::min(measured_.stamp, contact_stamp_);
      measured_.sample_id = std::min(measured_.sample_id, contact_clock_.sample_id());
      measured_.source_time_ns = contact_source_ns_;
      measured_.epoch = contact_epoch_;
    }
    if (!measured_.valid) {
      const auto joint_age_ms = measured_.stamp == rc::Time{} ? -1.0 :
          std::chrono::duration<double, std::milli>(now-measured_.stamp).count();
      const auto contact_age_ms = contact_stamp_ == rc::Time{} ? -1.0 :
          std::chrono::duration<double, std::milli>(now-contact_stamp_).count();
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
          "Gripper feedback invalid: joint_fresh=%s joint_sample=%lu joint_age_ms=%.1f contact_fresh=%s contact_age_ms=%.1f",
          joint_fresh ? "true" : "false", static_cast<unsigned long>(joint_clock_.sample_id()), joint_age_ms,
          contact_fresh ? "true" : "false", contact_age_ms);
    }
    return action_status;
  }
  void request_stop() override {
    channel_.stop();
  }
  rc::GripperFeedback feedback() const override { return measured_; }
 private:
  void send(double width, double effort) {
    if (!std::isfinite(width) || width < 0.0 || width > 0.08 || !std::isfinite(effort) || effort <= 0.0)
      throw std::invalid_argument("invalid Panda gripper command");
    control_msgs::action::ParallelGripperCommand::Goal goal;
    goal.command.name = {config_.finger_joint};
    goal.command.position = {width/2.0};  // Panda has one actuated finger and one symmetric mate.
    goal.command.effort = {effort};
    channel_.send(goal, [this](const auto& r) {
      const bool reached = r.result && r.result->reached_goal;
      const bool stalled = r.result && r.result->stalled;
      RCLCPP_INFO(node_->get_logger(), "Panda gripper action result: code=%d reached_goal=%s stalled=%s",
          static_cast<int>(r.code), reached ? "true" : "false", stalled ? "true" : "false");
      return reached || stalled;
    });
  }
  rclcpp::Node* node_;
  Config config_;
  ActionChannel<control_msgs::action::ParallelGripperCommand> channel_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr contact_;
  rclcpp::Subscription<robot_interfaces::msg::GraspContact>::SharedPtr stamped_contact_;
  sensor_msgs::msg::JointState joints_;
  rc::Time contact_stamp_{};
  uint64_t contact_source_ns_{0}, contact_epoch_{0}, contact_message_sequence_{0};
  MonotonicSampleClock joint_clock_, contact_clock_;
  bool contact_value_{false}, closing_{false}, sim_holding_{false};
  double commanded_width_{0.08};
  rc::GripperFeedback measured_;
};

// Gazebo ground-truth adapter. Truth is explicitly marked synthetic and
// advances only on new simulator timestamps; it never commands the robot.
class GazeboWorldAdapter final : public rc::ObjectLocator {
 public:
  GazeboWorldAdapter(rclcpp::Node* node, Config config, rc::DemoScene scene,
                     rc::WorldState* world_state = nullptr);
  std::string resource_id() const override { return "gazebo_world_state"; }
  std::optional<rc::Observation> locate(const std::string& id, rc::Time now) override;
  std::optional<rc::OutcomeEvidence> grasp(const std::string& object, rc::Time now);
  std::optional<rc::OutcomeEvidence> placement(const std::string& object, const std::string& target,
                                               rc::Time now);
 private:
  struct PoseSample {
    rc::Pose pose;
    rc::Time stamp{};
    rc::Pose previous_pose;
    rc::Time previous_stamp{};
    std::string frame_id;
    uint64_t sample_id{0}, source_ns{0}, previous_source_ns{0}, epoch{0};
    bool valid{false};
  };
  void on_poses(tf2_msgs::msg::TFMessage::ConstSharedPtr message);
  void activate_epoch(uint64_t epoch);
  const PoseSample* fresh_pose(const std::string& id, rc::Time now) const;
  std::string evidence_source() const;
  rclcpp::Node* node_;
  Config config_;
  rc::DemoScene scene_;
  rc::WorldState* world_state_{nullptr};
  std::map<std::string, PoseSample> poses_;
  std::vector<rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr> pose_subs_;
  std::map<std::string, MonotonicSampleClock> pose_clocks_;
  uint64_t epoch_{0}, world_sample_sequence_{0};
  rclcpp::Subscription<robot_interfaces::msg::GraspContact>::SharedPtr contact_sub_;
  MonotonicSampleClock contact_clock_;
  bool finger_contact_{false};
  rc::Time contact_stamp_{};
  uint64_t contact_source_ns_{0}, contact_epoch_{0}, contact_sequence_{0};
};

// Separate core interfaces avoid a Component diamond while sharing the same
// timestamped truth cache and Gazebo subscriptions.
class GazeboOutcomeAdapter final : public rc::ManipulationObserver {
 public:
  explicit GazeboOutcomeAdapter(std::shared_ptr<GazeboWorldAdapter> world) : world_(std::move(world)) {
    if (!world_) throw std::invalid_argument("Gazebo outcome adapter requires world state");
  }
  std::string resource_id() const override { return world_->resource_id(); }
  std::optional<rc::OutcomeEvidence> grasp(const std::string& object, rc::Time now) override {
    return world_->grasp(object, now);
  }
  std::optional<rc::OutcomeEvidence> placement(const std::string& object, const std::string& target,
                                                rc::Time now) override {
    return world_->placement(object, target, now);
  }
 private:
  std::shared_ptr<GazeboWorldAdapter> world_;
};

}  // namespace robot_ros_adapters
