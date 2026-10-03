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
  "srv/GetCatalog.srv")
ament_export_dependencies(rosidl_default_runtime)
ament_package()
