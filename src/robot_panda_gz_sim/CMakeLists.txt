cmake_minimum_required(VERSION 3.16)
project(robot_panda_gz_sim)
find_package(ament_cmake REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
install(DIRECTORY launch config worlds DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE PATTERN "*.pyc" EXCLUDE)
install(PROGRAMS scripts/gz_scene_sync.py DESTINATION lib/${PROJECT_NAME} RENAME gz_scene_sync)
if(BUILD_TESTING)
  enable_testing()
  add_test(NAME gazebo_sensor_timestamp_guard
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_sensor_timestamps.py)
endif()
ament_package()
