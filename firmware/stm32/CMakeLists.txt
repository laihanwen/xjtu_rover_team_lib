cmake_minimum_required(VERSION 3.22)

# This target exists to give CLion, VS Code and clangd the same source files,
# preprocessor symbols and include paths as the Keil project.  The deployable
# firmware is still built from MDK-ARM/Copy_cup.uvprojx.
project(ROV_UI_Model LANGUAGES C)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

set(ROV_CONTROL_SOURCES
    MDK-ARM/imu.c
    MDK-ARM/Mate.c
    MDK-ARM/PID.c
    MDK-ARM/RC.c
    MDK-ARM/Move.c
    MDK-ARM/AuvProtocol.c
    MDK-ARM/AuvSafety.c
    MDK-ARM/AuvLink.c
    MDK-ARM/AuvRcInput.c
    MDK-ARM/AuvMotionTarget.c
    MDK-ARM/AuvControlSource.c
    MDK-ARM/AuvGripper.c
    MDK-ARM/AuvCameraServo.c
    MDK-ARM/AuvRovDepth.c
    MDK-ARM/AuvDepth.c
)

set(ROV_GENERATED_SOURCES
    Core/Src/main.c
    Core/Src/gpio.c
    Core/Src/iwdg.c
    Core/Src/tim.c
    Core/Src/usart.c
    Core/Src/stm32f4xx_it.c
    Core/Src/stm32f4xx_hal_msp.c
    Core/Src/system_stm32f4xx.c
)

set(ROV_HAL_SOURCES
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_iwdg.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ex.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ramfunc.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma_ex.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr_ex.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_exti.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim_ex.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c
)

# An object library deliberately stops before linking.  It is a complete and
# fast syntax/code-model check; Keil remains authoritative for AXF/HEX output.
add_library(rov_ui_model OBJECT
    ${ROV_CONTROL_SOURCES}
    ${ROV_GENERATED_SOURCES}
    ${ROV_HAL_SOURCES}
)

target_compile_definitions(rov_ui_model PRIVATE
    USE_HAL_DRIVER
    STM32F405xx
)

target_include_directories(rov_ui_model PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/MDK-ARM
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32F4xx_HAL_Driver/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/CMSIS/Device/ST/STM32F4xx/Include
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/CMSIS/Include
)

target_compile_options(rov_ui_model PRIVATE
    -mcpu=cortex-m4
    -mfpu=fpv4-sp-d16
    -mfloat-abi=hard
    -Wall
    -Wextra
    -Wno-unused-parameter
)
