cmake_minimum_required(VERSION 3.20)
project(auv_stm32_bridge)

find_package(ament_cmake REQUIRED)
find_package(auv_core REQUIRED)
find_package(auv_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_library(serial_port INTERFACE)
target_link_libraries(serial_port INTERFACE auv_core::auv_core)

add_library(protocol INTERFACE)
target_link_libraries(protocol INTERFACE auv_core::auv_core)

add_library(motion_target INTERFACE)
target_link_libraries(motion_target INTERFACE auv_core::auv_core)

add_library(gripper_protocol INTERFACE)
target_link_libraries(gripper_protocol INTERFACE auv_core::auv_core)

add_executable(stm32_bridge_node src/stm32_bridge_node.cpp)
target_compile_features(stm32_bridge_node PUBLIC cxx_std_17)
target_link_libraries(stm32_bridge_node
  serial_port
  protocol
  motion_target
  gripper_protocol
  auv_interfaces::auv_interfaces__rosidl_generator_cpp
  auv_interfaces::auv_interfaces__rosidl_typesupport_cpp
  rclcpp::rclcpp
  ${geometry_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
)

install(TARGETS stm32_bridge_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_serial_port test/test_serial_port.cpp)
  if(TARGET test_serial_port)
    target_link_libraries(test_serial_port serial_port util)
  endif()

  ament_add_gtest(test_protocol test/test_protocol.cpp)
  if(TARGET test_protocol)
    target_link_libraries(test_protocol protocol)
  endif()

  ament_add_gtest(test_stream_parser test/test_stream_parser.cpp)
  if(TARGET test_stream_parser)
    target_link_libraries(test_stream_parser protocol)
  endif()

  ament_add_gtest(test_telemetry_decoder test/test_telemetry_decoder.cpp)
  if(TARGET test_telemetry_decoder)
    target_link_libraries(test_telemetry_decoder protocol)
  endif()

  ament_add_gtest(test_motion_target test/test_motion_target.cpp)
  if(TARGET test_motion_target)
    target_link_libraries(test_motion_target motion_target)
  endif()

  ament_add_gtest(test_gripper_protocol test/test_gripper_protocol.cpp)
  if(TARGET test_gripper_protocol)
    target_link_libraries(test_gripper_protocol gripper_protocol)
  endif()
endif()

ament_export_include_directories(include)
ament_export_dependencies(auv_core)
ament_package()
