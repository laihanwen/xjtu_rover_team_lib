cmake_minimum_required(VERSION 3.20)
project(auv_firmware_host_tests LANGUAGES C)

enable_testing()
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

if(MSVC)
    add_compile_options(/W4 /WX /UNDEBUG)
else()
    set(AUV_HOST_WARNINGS -Wall -Wextra -Werror -pedantic)
    add_compile_options(-UNDEBUG)
endif()

add_executable(test_auv_protocol
    test_auv_protocol.c
    ../MDK-ARM/AuvProtocol.c
)
target_include_directories(test_auv_protocol PRIVATE ../MDK-ARM)
target_compile_options(test_auv_protocol PRIVATE ${AUV_HOST_WARNINGS})
add_test(NAME auv_protocol COMMAND test_auv_protocol)

add_executable(test_auv_safety
    test_auv_safety.c
    ../MDK-ARM/AuvSafety.c
)
target_include_directories(test_auv_safety PRIVATE ../MDK-ARM)
target_compile_options(test_auv_safety PRIVATE ${AUV_HOST_WARNINGS})
add_test(NAME auv_safety COMMAND test_auv_safety)

add_executable(test_auv_rc_input
    test_auv_rc_input.c
    ../MDK-ARM/AuvRcInput.c
)
target_include_directories(test_auv_rc_input PRIVATE ../MDK-ARM)
target_compile_options(test_auv_rc_input PRIVATE ${AUV_HOST_WARNINGS})
add_test(NAME auv_rc_input COMMAND test_auv_rc_input)

add_executable(test_auv_motion_target
    test_auv_motion_target.c
    ../MDK-ARM/AuvMotionTarget.c
    ../MDK-ARM/AuvProtocol.c
)
target_include_directories(test_auv_motion_target PRIVATE ../MDK-ARM)
target_compile_options(test_auv_motion_target PRIVATE ${AUV_HOST_WARNINGS})
add_test(NAME auv_motion_target COMMAND test_auv_motion_target)

add_executable(test_auv_control_source
    test_auv_control_source.c
    ../MDK-ARM/AuvControlSource.c
)
target_include_directories(test_auv_control_source PRIVATE ../MDK-ARM)
target_compile_options(test_auv_control_source PRIVATE ${AUV_HOST_WARNINGS})
add_test(NAME auv_control_source COMMAND test_auv_control_source)

add_executable(test_auv_gripper
    test_auv_gripper.c
    ../MDK-ARM/AuvGripper.c
    ../MDK-ARM/AuvProtocol.c
)
target_include_directories(test_auv_gripper PRIVATE ../MDK-ARM)
target_compile_options(test_auv_gripper PRIVATE ${AUV_HOST_WARNINGS})
add_test(NAME auv_gripper COMMAND test_auv_gripper)

add_executable(test_auv_camera_servo
    test_auv_camera_servo.c
    ../MDK-ARM/AuvCameraServo.c
    ../MDK-ARM/AuvProtocol.c
)
target_include_directories(test_auv_camera_servo PRIVATE ../MDK-ARM)
target_compile_options(test_auv_camera_servo PRIVATE ${AUV_HOST_WARNINGS})
add_test(NAME auv_camera_servo COMMAND test_auv_camera_servo)

add_executable(test_auv_depth
    test_auv_depth.c
    ../MDK-ARM/AuvDepth.c
)
target_include_directories(test_auv_depth PRIVATE ../MDK-ARM)
target_compile_options(test_auv_depth PRIVATE ${AUV_HOST_WARNINGS})
add_test(NAME auv_depth COMMAND test_auv_depth)

add_executable(test_auv_rov_depth test_auv_rov_depth.c ../MDK-ARM/AuvRovDepth.c)
target_include_directories(test_auv_rov_depth PRIVATE ../MDK-ARM)
if(NOT MSVC)
    target_link_libraries(test_auv_rov_depth PRIVATE m)
endif()
add_test(NAME auv_rov_depth COMMAND test_auv_rov_depth)

add_executable(test_auv_angles test_auv_angles.c)
target_include_directories(test_auv_angles PRIVATE ../MDK-ARM)
target_compile_options(test_auv_angles PRIVATE ${AUV_HOST_WARNINGS})
add_test(NAME auv_angles COMMAND test_auv_angles)

add_executable(test_auv_h30_frame test_auv_h30_frame.c)
target_include_directories(test_auv_h30_frame PRIVATE ../MDK-ARM)
target_compile_options(test_auv_h30_frame PRIVATE ${AUV_HOST_WARNINGS})
add_test(NAME auv_h30_frame COMMAND test_auv_h30_frame)
add_executable(test_auv_deadband test_auv_deadband.c)
target_include_directories(test_auv_deadband PRIVATE ../MDK-ARM)
if(NOT MSVC)
    target_link_libraries(test_auv_deadband PRIVATE m)
endif()
add_test(NAME auv_deadband COMMAND test_auv_deadband)
add_executable(test_auv_priority test_auv_priority.c)
target_include_directories(test_auv_priority PRIVATE ../MDK-ARM)
if(NOT MSVC)
    target_link_libraries(test_auv_priority PRIVATE m)
endif()
add_test(NAME auv_priority COMMAND test_auv_priority)
add_executable(test_auv_water_pd test_auv_water_pd.c)
target_include_directories(test_auv_water_pd PRIVATE ../MDK-ARM)
if(NOT MSVC)
    target_link_libraries(test_auv_water_pd PRIVATE m)
endif()
add_test(NAME auv_water_pd COMMAND test_auv_water_pd)
