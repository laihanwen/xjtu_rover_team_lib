cmake_minimum_required(VERSION 3.20)
project(auv_vision)

find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgcodecs videoio)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_library(camera_source src/camera_source.cpp)
target_compile_features(camera_source PUBLIC cxx_std_17)
target_include_directories(camera_source PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(camera_source PUBLIC ${OpenCV_LIBS})

add_executable(camera_node src/camera_node.cpp)
target_compile_features(camera_node PUBLIC cxx_std_17)
target_link_libraries(camera_node
  camera_source
  cv_bridge::cv_bridge
  image_transport::image_transport
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
)

install(TARGETS camera_source camera_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_camera_source test/test_camera_source.cpp)
  if(TARGET test_camera_source)
    target_link_libraries(test_camera_source camera_source)
  endif()
endif()

ament_export_include_directories(include)
ament_export_libraries(camera_source)
ament_package()
