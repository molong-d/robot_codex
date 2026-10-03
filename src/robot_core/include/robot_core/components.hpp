#pragma once
#include "robot_core/core.hpp"
#include <optional>

namespace robot_core {
class ObjectLocator : public Component {
 public:
  std::string interface_id() const final { return "object_locator"; }
  virtual std::optional<Observation> locate(const std::string&, Time) = 0;
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
};
}  // namespace robot_core
