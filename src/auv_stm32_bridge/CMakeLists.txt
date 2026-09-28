cmake_minimum_required(VERSION 3.20)
project(auv_stm32_bridge)

find_package(ament_cmake REQUIRED)
find_package(auv_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(serial_port src/serial_port.cpp)
target_compile_features(serial_port PUBLIC cxx_std_17)
target_include_directories(serial_port PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_library(protocol src/protocol.c src/stream_parser.cpp src/telemetry_decoder.cpp)
target_compile_features(protocol PUBLIC c_std_11 cxx_std_17)
target_include_directories(protocol PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(stm32_bridge_node src/stm32_bridge_node.cpp)
target_compile_features(stm32_bridge_node PUBLIC cxx_std_17)
target_link_libraries(stm32_bridge_node
  serial_port
  protocol
  auv_interfaces::auv_interfaces__rosidl_generator_cpp
  auv_interfaces::auv_interfaces__rosidl_typesupport_cpp
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
)

install(TARGETS protocol serial_port stm32_bridge_node
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
endif()

ament_export_include_directories(include)
ament_export_libraries(protocol serial_port)
ament_package()
