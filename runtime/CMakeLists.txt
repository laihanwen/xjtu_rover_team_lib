find_package(yaml-cpp REQUIRED)
add_executable(auv_runtime src/runtime.cpp)
find_path(HTTPLIB_INCLUDE_DIR httplib.h HINTS ${HTTPLIB_ROOT}/usr/include)
find_library(HTTPLIB_LIBRARY NAMES cpp-httplib
  HINTS ${HTTPLIB_ROOT}/usr/lib/${CMAKE_LIBRARY_ARCHITECTURE}
        ${HTTPLIB_LIB_ROOT}/usr/lib/${CMAKE_LIBRARY_ARCHITECTURE})
if(HTTPLIB_INCLUDE_DIR AND HTTPLIB_LIBRARY)
  target_compile_definitions(auv_runtime PRIVATE AUV_HAVE_HTTPLIB=1)
  target_include_directories(auv_runtime PRIVATE ${HTTPLIB_INCLUDE_DIR})
  target_link_libraries(auv_runtime PRIVATE ${HTTPLIB_LIBRARY})
endif()
target_compile_features(auv_runtime PRIVATE cxx_std_17)
target_link_libraries(auv_runtime PRIVATE auv_core::auv_core yaml-cpp pthread)
add_executable(auvctl src/auvctl.cpp)
target_compile_features(auvctl PRIVATE cxx_std_17)
install(TARGETS auv_runtime auvctl RUNTIME DESTINATION bin)
install(DIRECTORY web/ DESTINATION share/auv-runtime/web)
install(FILES config/runtime.yaml DESTINATION etc/auv-runtime)
install(FILES deploy/auv-runtime.service DESTINATION lib/systemd/system)
if(BUILD_TESTING)
  find_package(Python3 COMPONENTS Interpreter REQUIRED)
  add_test(NAME runtime_pty
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_runtime.py
      $<TARGET_FILE:auv_runtime>)
  set_tests_properties(runtime_pty PROPERTIES
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR} TIMEOUT 15)
  execute_process(COMMAND ${Python3_EXECUTABLE} -c "import cv2, numpy"
    RESULT_VARIABLE AUV_REPLAY_DEPS OUTPUT_QUIET ERROR_QUIET)
  if(AUV_REPLAY_DEPS EQUAL 0)
    add_test(NAME offline_replay
      COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/offline_replay.py
        $<TARGET_FILE:auv_runtime>)
    set_tests_properties(offline_replay PROPERTIES
      WORKING_DIRECTORY ${CMAKE_SOURCE_DIR} TIMEOUT 20)
    add_test(NAME arm_simulation
      COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/arm_simulation.py
        $<TARGET_FILE:auv_runtime>)
    set_tests_properties(arm_simulation PROPERTIES
      WORKING_DIRECTORY ${CMAKE_SOURCE_DIR} TIMEOUT 20)
    add_test(NAME autonomous_simulation
      COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/arm_simulation.py
        $<TARGET_FILE:auv_runtime> autonomous)
    set_tests_properties(autonomous_simulation PROPERTIES
      WORKING_DIRECTORY ${CMAKE_SOURCE_DIR} TIMEOUT 20)
  endif()
endif()
