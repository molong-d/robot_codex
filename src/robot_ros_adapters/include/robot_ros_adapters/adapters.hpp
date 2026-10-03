#pragma once
#include "robot_core/components.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit_msgs/action/move_group.hpp>
#include <control_msgs/action/parallel_gripper_command.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <functional>

namespace robot_ros_adapters {
namespace rc = robot_core;
struct Config {
  std::string move_action{"/move_action"};
  std::string gripper_action{"/panda_hand_controller/gripper_cmd"};
  std::string group{"panda_arm"}, frame{"panda_link0"}, tip{"panda_hand"};
  std::vector<std::string> arm_joints{"panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"};
  std::string finger_joint{"panda_finger_joint1"};
  std::string joint_states_topic{"/joint_states"}, grasp_topic{"/grasp_detected"};
  std::string arm_resource{"panda_arm"}, gripper_resource{"panda_gripper"};
  bool simulation_grasp_detection{false};
  double velocity_scaling{0.2}, acceleration_scaling{0.2};
};

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

// Source ROS timestamps are checked before conversion to monotonic age. A cached
// ROS message is never made fresh merely by polling it again.
inline rc::Time monotonic_stamp(rclcpp::Node* node, const builtin_interfaces::msg::Time& stamp, rc::Time now) {
  const auto source = rclcpp::Time(stamp, node->get_clock()->get_clock_type());
  const double age = (node->now()-source).seconds();
  if (source.nanoseconds() == 0 || age < 0.0 || age > 0.5 || !std::isfinite(age)) return {};
  return now-std::chrono::duration_cast<rc::Clock::duration>(std::chrono::duration<double>(age));
}
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
        listener_(std::make_shared<tf2_ros::TransformListener>(*buffer_, node, false)) {
    sub_ = node->create_subscription<sensor_msgs::msg::JointState>(config_.joint_states_topic, 10,
        [this](sensor_msgs::msg::JointState::ConstSharedPtr s) { joints_ = *s; });
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
      const auto js_stamp = monotonic_stamp(node_, joints_.header.stamp, now);
      const auto tf_stamp = monotonic_stamp(node_, t.header.stamp, now);
      measured_.frame_id = config_.frame;
      measured_.pose = {t.transform.translation.x, t.transform.translation.y, t.transform.translation.z,
                        t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w};
      measured_.stamp = std::min(js_stamp, tf_stamp);
      measured_.valid = js_stamp != rc::Time{} && tf_stamp != rc::Time{} && rc::valid_pose(measured_.pose);
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
};

class ParallelGripper final : public rc::Gripper {
 public:
  ParallelGripper(rclcpp::Node* node, Config config)
      : node_(node), config_(std::move(config)), channel_(node, config_.gripper_action) {
    sub_ = node->create_subscription<sensor_msgs::msg::JointState>(config_.joint_states_topic, 10,
        [this](sensor_msgs::msg::JointState::ConstSharedPtr s) { joints_ = *s; });
    contact_ = node->create_subscription<std_msgs::msg::Bool>(config_.grasp_topic, 10,
        [this](std_msgs::msg::Bool::ConstSharedPtr m) { contact_value_ = m->data; contact_stamp_ = rc::Clock::now(); });
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
    auto it = std::find(joints_.name.begin(), joints_.name.end(), config_.finger_joint);
    if (it != joints_.name.end()) {
      const auto i = static_cast<size_t>(it-joints_.name.begin());
      if (i < joints_.position.size()) {
        measured_.width_m = 2.0*joints_.position[i];
        measured_.stamp = monotonic_stamp(node_, joints_.header.stamp, now);
        measured_.valid = measured_.stamp != rc::Time{} && std::isfinite(measured_.width_m) && measured_.width_m >= 0.0;
        measured_.stopped = measured_.valid && stationary(joints_, {config_.finger_joint});
      }
    }
    if (config_.simulation_grasp_detection) {
      // Explicit synthetic contact for GenericSystem demo only. No contact physics.
      if (measured_.valid && measured_.stopped && channel_.status() != rc::Status::idle &&
          std::abs(measured_.width_m-commanded_width_) <= 0.002) sim_holding_ = closing_;
      measured_.grasp_detected = sim_holding_;
    } else {
      measured_.valid = measured_.valid && rc::fresh(contact_stamp_, now, std::chrono::milliseconds(500));
      measured_.grasp_detected = contact_value_;
    }
    return channel_.status();
  }
  void request_stop() override { channel_.stop(); }
  rc::GripperFeedback feedback() const override { return measured_; }
 private:
  void send(double width, double effort) {
    if (!std::isfinite(width) || width < 0.0 || width > 0.08 || !std::isfinite(effort) || effort <= 0.0)
      throw std::invalid_argument("invalid Panda gripper command");
    control_msgs::action::ParallelGripperCommand::Goal goal;
    goal.command.name = {config_.finger_joint};
    goal.command.position = {width/2.0};  // Panda has two symmetric fingers.
    goal.command.effort = {effort};
    channel_.send(goal, [](const auto& r) { return r.result->reached_goal || r.result->stalled; });
  }
  rclcpp::Node* node_;
  Config config_;
  ActionChannel<control_msgs::action::ParallelGripperCommand> channel_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr contact_;
  sensor_msgs::msg::JointState joints_;
  rc::Time contact_stamp_{};
  bool contact_value_{false}, closing_{false}, sim_holding_{false};
  double commanded_width_{0.08};
  rc::GripperFeedback measured_;
};

// Known demo coordinates are target configuration, not visual detections.
class DemoTargets final : public rc::ObjectLocator {
 public:
  explicit DemoTargets(std::string frame) : frame_(std::move(frame)) {}
  std::string resource_id() const override { return "panda_demo_target_catalog"; }
  std::optional<rc::Observation> locate(const std::string& id, rc::Time now) override {
    if (id == "workpiece") return rc::Observation{id, frame_, {0.4, 0.1, 0.4, 1.0, 0.0, 0.0, 0.0}, now, true};
    if (id == "tray") return rc::Observation{id, frame_, {0.45, -0.15, 0.4, 1.0, 0.0, 0.0, 0.0}, now, true};
    return std::nullopt;
  }
 private:
  std::string frame_;
};
}  // namespace robot_ros_adapters
