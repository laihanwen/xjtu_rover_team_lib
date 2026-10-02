cmake_minimum_required(VERSION 3.20)
project(auv_mission)

find_package(ament_cmake REQUIRED)
find_package(auv_interfaces REQUIRED)
find_package(rclcpp REQUIRED)

add_library(mission_fsm src/mission_fsm.cpp)
target_compile_features(mission_fsm PUBLIC cxx_std_17)
target_include_directories(mission_fsm PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_library(mission_manager src/mission_node.cpp)
target_compile_features(mission_manager PUBLIC cxx_std_17)
target_include_directories(mission_manager PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(mission_manager PUBLIC
  mission_fsm
  rclcpp::rclcpp
  ${auv_interfaces_TARGETS}
)

add_executable(mission_manager_node src/mission_main.cpp)
target_compile_features(mission_manager_node PUBLIC cxx_std_17)
target_link_libraries(mission_manager_node mission_manager rclcpp::rclcpp)

add_executable(mission_virtual_test src/mission_virtual_test.cpp)
target_compile_features(mission_virtual_test PUBLIC cxx_std_17)
target_link_libraries(mission_virtual_test
  mission_manager
  rclcpp::rclcpp
  ${auv_interfaces_TARGETS}
)

install(TARGETS
  mission_fsm mission_manager mission_manager_node mission_virtual_test
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_mission_fsm test/test_mission_fsm.cpp)
  if(TARGET test_mission_fsm)
    target_link_libraries(test_mission_fsm mission_fsm)
  endif()

  add_test(NAME mission_virtual_system_test COMMAND mission_virtual_test)
  set_tests_properties(mission_virtual_system_test PROPERTIES TIMEOUT 15)
endif()

ament_export_include_directories(include)
ament_export_libraries(mission_fsm mission_manager)
ament_package()
