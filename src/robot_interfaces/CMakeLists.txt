cmake_minimum_required(VERSION 3.16)
project(robot_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/ExecuteTask.action"
  "action/ExecutePlan.action"
  "msg/SkillStep.msg"
  "msg/SkillInput.msg"
  "msg/ComponentRequirement.msg"
  "msg/SkillImplementation.msg"
  "msg/SkillInfo.msg"
  "msg/TaskInfo.msg"
  "msg/StateTransition.msg"
  "msg/SkillExecutionRecord.msg"
  "msg/ObservationSnapshot.msg"
  "msg/EntityLocation.msg"
  "msg/ResourceLease.msg"
  "msg/WorldSnapshot.msg"
  "msg/ExecutionRecord.msg"
  "srv/GetCatalog.srv"
  "srv/GetRuntimeState.srv"
  "srv/GetExecution.srv")
ament_export_dependencies(rosidl_default_runtime)
ament_package()
