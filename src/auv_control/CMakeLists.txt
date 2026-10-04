cmake_minimum_required(VERSION 3.20)
project(auv_control)

find_package(ament_cmake REQUIRED)
find_package(auv_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_library(route_executor_core src/route_executor.cpp)
target_compile_features(route_executor_core PUBLIC cxx_std_17)
target_include_directories(route_executor_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(route_executor_core PUBLIC ${auv_interfaces_TARGETS})

add_library(route_executor_node_lib src/route_executor_node.cpp)
target_compile_features(route_executor_node_lib PUBLIC cxx_std_17)
target_include_directories(route_executor_node_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(route_executor_node_lib
  route_executor_core
  rclcpp::rclcpp
  ${auv_interfaces_TARGETS}
  ${geometry_msgs_TARGETS}
  ${std_msgs_TARGETS})

add_executable(route_executor_node src/route_executor_main.cpp)
target_compile_features(route_executor_node PUBLIC cxx_std_17)
target_link_libraries(route_executor_node route_executor_node_lib rclcpp::rclcpp)

install(TARGETS route_executor_core route_executor_node_lib route_executor_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_add_gtest(test_route_executor test/test_route_executor.cpp)
  target_link_libraries(test_route_executor route_executor_core)
  target_link_libraries(test_route_executor ${auv_interfaces_TARGETS})
  ament_add_gtest(test_route_executor_node test/test_route_executor_node.cpp)
  target_link_libraries(test_route_executor_node
    route_executor_node_lib
    rclcpp::rclcpp
    ${auv_interfaces_TARGETS}
    ${geometry_msgs_TARGETS})
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(route_executor_core route_executor_node_lib)
ament_export_dependencies(auv_interfaces)
ament_package()
