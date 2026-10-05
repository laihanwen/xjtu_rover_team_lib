cmake_minimum_required(VERSION 3.20)
project(auv_firmware_host_tests LANGUAGES C)

enable_testing()
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

add_executable(test_auv_protocol
    test_auv_protocol.c
    ../MDK-ARM/AuvProtocol.c
)
target_include_directories(test_auv_protocol PRIVATE ../MDK-ARM)
target_compile_options(test_auv_protocol PRIVATE -Wall -Wextra -Werror -pedantic)
add_test(NAME auv_protocol COMMAND test_auv_protocol)

add_executable(test_auv_safety
    test_auv_safety.c
    ../MDK-ARM/AuvSafety.c
)
target_include_directories(test_auv_safety PRIVATE ../MDK-ARM)
target_compile_options(test_auv_safety PRIVATE -Wall -Wextra -Werror -pedantic)
add_test(NAME auv_safety COMMAND test_auv_safety)

add_executable(test_auv_rc_input
    test_auv_rc_input.c
    ../MDK-ARM/AuvRcInput.c
)
target_include_directories(test_auv_rc_input PRIVATE ../MDK-ARM)
target_compile_options(test_auv_rc_input PRIVATE -Wall -Wextra -Werror -pedantic)
add_test(NAME auv_rc_input COMMAND test_auv_rc_input)

add_executable(test_auv_motion_target
    test_auv_motion_target.c
    ../MDK-ARM/AuvMotionTarget.c
    ../MDK-ARM/AuvProtocol.c
)
target_include_directories(test_auv_motion_target PRIVATE ../MDK-ARM)
target_compile_options(test_auv_motion_target PRIVATE -Wall -Wextra -Werror -pedantic)
add_test(NAME auv_motion_target COMMAND test_auv_motion_target)

add_executable(test_auv_control_source
    test_auv_control_source.c
    ../MDK-ARM/AuvControlSource.c
)
target_include_directories(test_auv_control_source PRIVATE ../MDK-ARM)
target_compile_options(test_auv_control_source PRIVATE -Wall -Wextra -Werror -pedantic)
add_test(NAME auv_control_source COMMAND test_auv_control_source)

add_executable(test_auv_gripper
    test_auv_gripper.c
    ../MDK-ARM/AuvGripper.c
    ../MDK-ARM/AuvProtocol.c
)
target_include_directories(test_auv_gripper PRIVATE ../MDK-ARM)
target_compile_options(test_auv_gripper PRIVATE -Wall -Wextra -Werror -pedantic)
add_test(NAME auv_gripper COMMAND test_auv_gripper)

add_executable(test_auv_camera_servo
    test_auv_camera_servo.c
    ../MDK-ARM/AuvCameraServo.c
    ../MDK-ARM/AuvProtocol.c
)
target_include_directories(test_auv_camera_servo PRIVATE ../MDK-ARM)
target_compile_options(test_auv_camera_servo PRIVATE -Wall -Wextra -Werror -pedantic)
add_test(NAME auv_camera_servo COMMAND test_auv_camera_servo)
