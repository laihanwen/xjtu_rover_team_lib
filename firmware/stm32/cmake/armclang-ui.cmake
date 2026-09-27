set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR cortex-m4)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# Prefer KEIL_ROOT, then the common Keil installation locations.  Avoiding a
# generated C:/Users/... path keeps the project independent of its old PC.
if(DEFINED ENV{KEIL_ROOT} AND
   EXISTS "$ENV{KEIL_ROOT}/ARM/ARMCLANG/bin/armclang.exe")
    set(ARMCLANG_EXECUTABLE
        "$ENV{KEIL_ROOT}/ARM/ARMCLANG/bin/armclang.exe")
elseif(EXISTS "D:/Keil_v5/ARM/ARMCLANG/bin/armclang.exe")
    set(ARMCLANG_EXECUTABLE "D:/Keil_v5/ARM/ARMCLANG/bin/armclang.exe")
elseif(EXISTS "C:/Keil_v5/ARM/ARMCLANG/bin/armclang.exe")
    set(ARMCLANG_EXECUTABLE "C:/Keil_v5/ARM/ARMCLANG/bin/armclang.exe")
elseif(EXISTS "E:/Keil_v5/ARM/ARMCLANG/bin/armclang.exe")
    set(ARMCLANG_EXECUTABLE "E:/Keil_v5/ARM/ARMCLANG/bin/armclang.exe")
else()
    message(FATAL_ERROR
        "Keil ARMClang was not found. Set KEIL_ROOT to the Keil_v5 directory.")
endif()

set(CMAKE_C_COMPILER "${ARMCLANG_EXECUTABLE}" CACHE FILEPATH "" FORCE)
set(CMAKE_C_COMPILER_TARGET arm-arm-none-eabi)
