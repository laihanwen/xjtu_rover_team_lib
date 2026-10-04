cmake_minimum_required(VERSION 3.20)
project(auv_core VERSION 0.1.0 LANGUAGES C CXX)

find_package(OpenCV REQUIRED COMPONENTS aruco calib3d core imgcodecs imgproc videoio)
include(CMakePackageConfigHelpers)
include(GNUInstallDirs)

add_library(auv_core
  ../auv_vision/src/camera_source.cpp
  ../auv_vision/src/apriltag_detector.cpp
  ../auv_vision/src/cone_detector.cpp
  ../auv_mapping/src/grid_mapper.cpp
  ../auv_planning/src/grid_planner.cpp
  ../auv_mission/src/mission_fsm.cpp
  ../auv_control/src/route_executor.cpp
  ../auv_stm32_bridge/src/protocol.c
  ../auv_stm32_bridge/src/stream_parser.cpp
  ../auv_stm32_bridge/src/serial_port.cpp
  ../auv_stm32_bridge/src/motion_target.cpp
  ../auv_stm32_bridge/src/telemetry_decoder.cpp
  ../auv_stm32_bridge/src/gripper_protocol.cpp
  src/status_decoder.cpp
  src/semantic_map.cpp)
add_library(auv_core::auv_core ALIAS auv_core)
target_compile_features(auv_core PUBLIC cxx_std_17)
target_include_directories(auv_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../auv_vision/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../auv_mapping/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../auv_planning/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../auv_mission/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../auv_control/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../auv_stm32_bridge/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(auv_core PUBLIC ${OpenCV_LIBS})
set_target_properties(auv_core PROPERTIES EXPORT_NAME auv_core)
set_target_properties(auv_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

install(TARGETS auv_core EXPORT auv_coreTargets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})
foreach(package auv_core auv_vision auv_mapping auv_planning auv_mission auv_control auv_stm32_bridge)
  install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../${package}/include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR} OPTIONAL)
endforeach()
install(EXPORT auv_coreTargets NAMESPACE auv_core::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/auv_core)
configure_package_config_file(cmake/auv_coreConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/auv_coreConfig.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/auv_core)
write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/auv_coreConfigVersion.cmake
  VERSION ${PROJECT_VERSION} COMPATIBILITY SameMajorVersion)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/auv_coreConfig.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/auv_coreConfigVersion.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/auv_core)

if(BUILD_TESTING)
  enable_testing()
  add_executable(test_core test/test_core.cpp)
  target_link_libraries(test_core PRIVATE auv_core)
  add_test(NAME test_core COMMAND test_core)
endif()

if(NOT AUV_NATIVE_BUILD)
  find_package(ament_cmake REQUIRED)
  ament_export_targets(auv_coreTargets HAS_LIBRARY_TARGET)
  ament_export_dependencies(OpenCV)
  ament_package()
endif()
