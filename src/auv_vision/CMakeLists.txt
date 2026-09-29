cmake_minimum_required(VERSION 3.20)
project(auv_vision)

find_package(ament_cmake REQUIRED)
find_package(auv_interfaces REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS aruco calib3d core imgcodecs imgproc videoio)
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

add_library(apriltag_detector src/apriltag_detector.cpp)
target_compile_features(apriltag_detector PUBLIC cxx_std_17)
target_include_directories(apriltag_detector PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(apriltag_detector PUBLIC ${OpenCV_LIBS})

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

add_executable(apriltag_detector_node src/apriltag_detector_node.cpp)
target_compile_features(apriltag_detector_node PUBLIC cxx_std_17)
target_link_libraries(apriltag_detector_node
  apriltag_detector
  cv_bridge::cv_bridge
  ${geometry_msgs_TARGETS}
  image_transport::image_transport
  rclcpp::rclcpp
  ${auv_interfaces_TARGETS}
  ${sensor_msgs_TARGETS}
)

install(TARGETS camera_source apriltag_detector camera_node apriltag_detector_node
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

  ament_add_gtest(test_apriltag_detector test/test_apriltag_detector.cpp)
  if(TARGET test_apriltag_detector)
    target_link_libraries(test_apriltag_detector apriltag_detector)
  endif()
endif()

ament_export_include_directories(include)
ament_export_libraries(camera_source apriltag_detector)
ament_package()
