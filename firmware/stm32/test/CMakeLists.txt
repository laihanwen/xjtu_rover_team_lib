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
