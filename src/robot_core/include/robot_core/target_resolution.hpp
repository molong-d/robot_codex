#pragma once
#include "robot_core/components.hpp"

namespace robot_core {
struct StaticCalibration {
  std::string id, source_frame, base_frame, tool_frame;
  Pose source_in_base, grasp_tool_in_object, placement_tool_in_target;
  bool synthetic{true};
};

// Explicit, immutable transforms for one source/base/tool combination. This is
// not a calibration estimator, TF cache, IK solver or approach-path planner.
class StaticMotionTargetResolver final : public MotionTargetResolver {
 public:
  explicit StaticMotionTargetResolver(StaticCalibration calibration) : calibration_(std::move(calibration)) {
    if (!valid_id(calibration_.id) || calibration_.source_frame.empty() ||
        calibration_.base_frame.empty() || calibration_.tool_frame.empty() ||
        !valid_pose(calibration_.source_in_base) || !valid_pose(calibration_.grasp_tool_in_object) ||
        !valid_pose(calibration_.placement_tool_in_target))
      throw std::invalid_argument("invalid static target calibration");
  }
  std::string resource_id() const override { return "static_calibration_" + calibration_.id; }
  std::optional<ResolvedMotionTarget> resolve(const Observation& input, TargetPurpose purpose) override {
    if (!input.valid || !valid_id(input.object_id) || input.meaning != PoseMeaning::object_pose ||
        input.frame_id != calibration_.source_frame || !valid_pose(input.pose) ||
        (purpose != TargetPurpose::grasp && purpose != TargetPurpose::placement)) return std::nullopt;
    auto target = input;  // retain original capture time, source and quality
    try {
      target.pose = compose_pose(compose_pose(calibration_.source_in_base, input.pose),
          purpose == TargetPurpose::grasp ? calibration_.grasp_tool_in_object : calibration_.placement_tool_in_target);
    } catch (const std::invalid_argument&) { return std::nullopt; }
    target.frame_id = calibration_.base_frame;
    target.meaning = PoseMeaning::motion_target;
    target.evidence.synthetic = input.evidence.synthetic || calibration_.synthetic;
    return ResolvedMotionTarget{target, calibration_.id, calibration_.tool_frame};
  }
 private:
  const StaticCalibration calibration_;
};
}  // namespace robot_core
