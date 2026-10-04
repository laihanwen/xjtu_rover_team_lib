cmake_minimum_required(VERSION 3.20)
project(auv_mapping)

find_package(ament_cmake REQUIRED)
find_package(auv_core REQUIRED)
find_package(auv_interfaces REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS calib3d core imgproc)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_library(grid_mapper INTERFACE)
target_link_libraries(grid_mapper INTERFACE auv_core::auv_core)

add_library(semantic_mapper src/semantic_mapper_node.cpp)
target_compile_features(semantic_mapper PUBLIC cxx_std_17)
target_include_directories(semantic_mapper PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(semantic_mapper PUBLIC
  grid_mapper
  cv_bridge::cv_bridge
  image_transport::image_transport
  rclcpp::rclcpp
  ${auv_interfaces_TARGETS}
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
)

add_executable(semantic_mapper_node src/semantic_mapper_main.cpp)
target_compile_features(semantic_mapper_node PUBLIC cxx_std_17)
target_link_libraries(semantic_mapper_node semantic_mapper rclcpp::rclcpp)

install(TARGETS semantic_mapper semantic_mapper_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_grid_mapper test/test_grid_mapper.cpp)
  if(TARGET test_grid_mapper)
    target_link_libraries(test_grid_mapper grid_mapper)
  endif()

  ament_add_gtest(test_semantic_mapper_node test/test_semantic_mapper_node.cpp)
  if(TARGET test_semantic_mapper_node)
    target_link_libraries(test_semantic_mapper_node
      semantic_mapper
      cv_bridge::cv_bridge
      rclcpp::rclcpp
      ${auv_interfaces_TARGETS}
      ${sensor_msgs_TARGETS}
    )
  endif()
endif()

ament_export_include_directories(include)
ament_export_dependencies(auv_core)
ament_export_libraries(semantic_mapper)
ament_package()
