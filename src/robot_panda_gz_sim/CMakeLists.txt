cmake_minimum_required(VERSION 3.16)
project(robot_panda_gz_sim)
find_package(ament_cmake REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
install(DIRECTORY launch config worlds DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE PATTERN "*.pyc" EXCLUDE)
install(PROGRAMS scripts/gz_scene_sync.py DESTINATION lib/${PROJECT_NAME} RENAME gz_scene_sync)
install(FILES scripts/scene_state.py scripts/source_time.py scripts/contact_semantics.py DESTINATION lib/${PROJECT_NAME})
if(BUILD_TESTING)
  enable_testing()
  add_test(NAME gazebo_sensor_timestamp_guard
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_sensor_timestamps.py)
  add_test(NAME gazebo_scene_state_guard
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_scene_state.py)
  add_test(NAME gazebo_trial_audit_guard
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_gazebo_trials.py)
  add_test(NAME gazebo_contact_semantics
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_contact_semantics.py)
  add_test(NAME gazebo_release_window_audit
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_evidence_release_window.py)
  add_test(NAME gazebo_offline_evidence_roundtrip
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_offline_evidence.py)
endif()
ament_package()
