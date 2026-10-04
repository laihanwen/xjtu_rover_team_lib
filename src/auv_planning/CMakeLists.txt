cmake_minimum_required(VERSION 3.20)
project(auv_planning)

find_package(ament_cmake REQUIRED)
find_package(auv_core REQUIRED)
find_package(auv_interfaces REQUIRED)
find_package(rclcpp REQUIRED)

add_library(grid_planner INTERFACE)
target_link_libraries(grid_planner INTERFACE auv_core::auv_core)

add_library(planner src/planner_node.cpp)
target_compile_features(planner PUBLIC cxx_std_17)
target_include_directories(planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(planner PUBLIC
  grid_planner
  rclcpp::rclcpp
  ${auv_interfaces_TARGETS}
)

add_executable(planner_node src/planner_main.cpp)
target_compile_features(planner_node PUBLIC cxx_std_17)
target_link_libraries(planner_node planner rclcpp::rclcpp)

install(TARGETS planner planner_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_grid_planner test/test_grid_planner.cpp)
  if(TARGET test_grid_planner)
    target_link_libraries(test_grid_planner grid_planner)
  endif()

  ament_add_gtest(test_planner_node test/test_planner_node.cpp)
  if(TARGET test_planner_node)
    target_link_libraries(test_planner_node
      planner
      rclcpp::rclcpp
      ${auv_interfaces_TARGETS}
    )
  endif()
endif()

ament_export_include_directories(include)
ament_export_dependencies(auv_core)
ament_export_libraries(planner)
ament_package()
