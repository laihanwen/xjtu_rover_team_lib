cmake_minimum_required(VERSION 3.20)
project(auv_interfaces)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

set(msg_files
  "msg/AprilTagDetection.msg"
  "msg/AprilTagDetectionArray.msg"
  "msg/ConeDetection.msg"
  "msg/ConeDetectionArray.msg"
  "msg/Detection2D.msg"
  "msg/Depth.msg"
  "msg/GridCell.msg"
  "msg/GridPose.msg"
  "msg/GripperStatus.msg"
  "msg/MissionState.msg"
  "msg/ObjectDetectionArray.msg"
  "msg/PlannedRoute.msg"
  "msg/RouteExecutionState.msg"
  "msg/SemanticCell.msg"
  "msg/SemanticMap.msg"
  "msg/Stm32Status.msg"
  "msg/ValveDetection.msg"
)

set(srv_files
  "srv/MissionCommand.srv"
  "srv/SetArmed.srv"
  "srv/SetGripper.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
