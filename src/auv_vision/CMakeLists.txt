cmake_minimum_required(VERSION 3.20)
project(auv_vision)

find_package(ament_cmake REQUIRED)
find_package(auv_core REQUIRED)
find_package(auv_interfaces REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS aruco calib3d core dnn imgcodecs imgproc videoio)
find_package(OpenSSL REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Threads REQUIRED)

add_library(camera_source INTERFACE)
target_link_libraries(camera_source INTERFACE auv_core::auv_core)

add_library(apriltag_detector INTERFACE)
target_link_libraries(apriltag_detector INTERFACE auv_core::auv_core)

add_library(cone_detector INTERFACE)
target_link_libraries(cone_detector INTERFACE auv_core::auv_core)

add_library(yolo_detector src/yolo_detector.cpp)
target_compile_features(yolo_detector PUBLIC cxx_std_17)
target_include_directories(yolo_detector PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(yolo_detector PUBLIC ${OpenCV_LIBS} OpenSSL::Crypto)

add_library(valve_detector src/valve_detector.cpp)
target_compile_features(valve_detector PUBLIC cxx_std_17)
target_include_directories(valve_detector PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(valve_detector PUBLIC ${OpenCV_LIBS})

add_library(cone_detector_node src/cone_detector_node.cpp)
target_compile_features(cone_detector_node PUBLIC cxx_std_17)
target_include_directories(cone_detector_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(cone_detector_node PUBLIC
  cone_detector
  cv_bridge::cv_bridge
  image_transport::image_transport
  rclcpp::rclcpp
  ${auv_interfaces_TARGETS}
  ${sensor_msgs_TARGETS}
)

add_executable(camera_node src/camera_node.cpp)
target_compile_features(camera_node PUBLIC cxx_std_17)
target_link_libraries(camera_node
  camera_source
  cv_bridge::cv_bridge
  image_transport::image_transport
  rclcpp::rclcpp
  Threads::Threads
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

add_executable(cone_detector_node_executable src/cone_detector_main.cpp)
set_target_properties(cone_detector_node_executable PROPERTIES OUTPUT_NAME cone_detector_node)
target_compile_features(cone_detector_node_executable PUBLIC cxx_std_17)
target_link_libraries(cone_detector_node_executable cone_detector_node rclcpp::rclcpp)

add_library(cucumber_detector_node src/cucumber_detector_node.cpp)
target_compile_features(cucumber_detector_node PUBLIC cxx_std_17)
target_include_directories(cucumber_detector_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(cucumber_detector_node PUBLIC
  yolo_detector
  cv_bridge::cv_bridge
  image_transport::image_transport
  rclcpp::rclcpp
  ${auv_interfaces_TARGETS}
  ${sensor_msgs_TARGETS}
)

add_executable(cucumber_detector_node_executable src/cucumber_detector_main.cpp)
set_target_properties(
  cucumber_detector_node_executable PROPERTIES OUTPUT_NAME cucumber_detector_node)
target_compile_features(cucumber_detector_node_executable PUBLIC cxx_std_17)
target_link_libraries(cucumber_detector_node_executable cucumber_detector_node rclcpp::rclcpp)

add_library(valve_detector_node src/valve_detector_node.cpp)
target_compile_features(valve_detector_node PUBLIC cxx_std_17)
target_include_directories(valve_detector_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(valve_detector_node PUBLIC
  valve_detector
  cv_bridge::cv_bridge
  image_transport::image_transport
  rclcpp::rclcpp
  ${auv_interfaces_TARGETS}
  ${sensor_msgs_TARGETS}
)

add_executable(valve_detector_node_executable src/valve_detector_main.cpp)
set_target_properties(
  valve_detector_node_executable PROPERTIES OUTPUT_NAME valve_detector_node)
target_compile_features(valve_detector_node_executable PUBLIC cxx_std_17)
target_link_libraries(valve_detector_node_executable valve_detector_node rclcpp::rclcpp)

install(TARGETS
  cone_detector_node yolo_detector valve_detector
  cucumber_detector_node camera_node apriltag_detector_node cone_detector_node_executable
  cucumber_detector_node_executable valve_detector_node valve_detector_node_executable
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

  ament_add_gtest(test_cone_detector test/test_cone_detector.cpp)
  if(TARGET test_cone_detector)
    target_link_libraries(test_cone_detector cone_detector)
  endif()

  ament_add_gtest(test_cone_detector_node test/test_cone_detector_node.cpp)
  if(TARGET test_cone_detector_node)
    target_link_libraries(test_cone_detector_node cone_detector_node rclcpp::rclcpp)
  endif()

  ament_add_gtest(test_yolo_detector test/test_yolo_detector.cpp)
  if(TARGET test_yolo_detector)
    target_link_libraries(test_yolo_detector yolo_detector)
  endif()

  ament_add_gtest(test_cucumber_detector_node test/test_cucumber_detector_node.cpp)
  if(TARGET test_cucumber_detector_node)
    target_link_libraries(
      test_cucumber_detector_node cucumber_detector_node rclcpp::rclcpp)
  endif()

  ament_add_gtest(test_valve_detector test/test_valve_detector.cpp)
  if(TARGET test_valve_detector)
    target_link_libraries(test_valve_detector valve_detector)
  endif()

  ament_add_gtest(test_valve_detector_node test/test_valve_detector_node.cpp)
  if(TARGET test_valve_detector_node)
    target_link_libraries(test_valve_detector_node valve_detector_node rclcpp::rclcpp)
  endif()
endif()

ament_export_include_directories(include)
ament_export_dependencies(auv_core)
ament_export_libraries(
  cone_detector_node yolo_detector
  cucumber_detector_node valve_detector valve_detector_node)
ament_package()
