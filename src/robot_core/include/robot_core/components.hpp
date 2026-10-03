#pragma once
#include "robot_core/core.hpp"
#include <optional>

namespace robot_core {
class ObjectLocator : public Component {
 public:
  std::string interface_id() const final { return "object_locator"; }
  unsigned interface_version() const override { return 2; }
  virtual std::optional<Observation> locate(const std::string&, Time) = 0;
};
// Supplies cached sensor evidence; never commands motion or retries a grasp.
class ManipulationObserver : public Component {
 public:
  std::string interface_id() const final { return "manipulation_observer"; }
  virtual std::optional<OutcomeEvidence> grasp(const std::string& object, Time now) = 0;
  virtual std::optional<OutcomeEvidence> placement(const std::string& object, const std::string& target, Time now) = 0;
};
struct CartesianTarget {
  std::string id;
  std::string frame_id;
  Pose pose;
  PoseTolerance tolerance;
};
struct MotionFeedback {
  std::string frame_id;
  Pose pose;
  Time stamp{};
  bool valid{false};
  bool stopped{false};
  uint64_t sample_id{0};  // oldest contributing source stamp in the demo adapters
};
class ArmMotion : public Component {
 public:
  std::string interface_id() const final { return "arm_motion"; }
  unsigned interface_version() const final { return 2; }
  virtual void begin_move(const CartesianTarget&) = 0;
  // Terminal status alone never proves physical stop. Feedback is checked separately.
  virtual Status poll(Time now) = 0;
  virtual void request_stop() = 0;
  virtual MotionFeedback feedback() const = 0;
};
struct GraspCommand { double width_m{0.0}; double max_effort_n{0.0}; };
struct GripperFeedback {
  double width_m{0.0};
  Time stamp{};
  bool valid{false};
  bool stopped{false};
  bool grasp_detected{false};
  uint64_t sample_id{0};  // producer sample identity, unchanged for cached ROS messages
};
class Gripper : public Component {
 public:
  std::string interface_id() const final { return "gripper"; }
  unsigned interface_version() const final { return 2; }
  virtual void begin_grasp(const GraspCommand&) = 0;
  virtual void begin_release(double width_m) = 0;
  virtual Status poll(Time now) = 0;
  virtual void request_stop() = 0;
  virtual GripperFeedback feedback() const = 0;
};
struct ManipulationPolicy {
  std::string frame_id{"base_link"};
  PoseTolerance tolerance;
  std::chrono::milliseconds observation_max_age{2000};
  std::chrono::milliseconds feedback_max_age{500};
  GraspCommand grasp{0.01, 20.0};
  double release_width_m{0.08};
  double gripper_tolerance_m{0.002};
  double minimum_observation_quality{0.8};
  bool allow_synthetic{false};
};
struct VerificationPolicy {
  EvidencePolicy evidence;
  std::chrono::milliseconds stable_duration{100};
  unsigned minimum_samples{3};
};
}  // namespace robot_core
