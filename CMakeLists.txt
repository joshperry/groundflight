# GroundFlight - RC Car Gyro Stabilizer
# Target: RadioMaster Nexus (STM32F722RET6)

cmake_minimum_required(VERSION 3.20)

# Must set toolchain before project()
set(CMAKE_TOOLCHAIN_FILE ${CMAKE_CURRENT_SOURCE_DIR}/cmake/arm-none-eabi.cmake)

project(groundflight
    VERSION 0.1.0
    LANGUAGES C ASM
)

# =============================================================================
# Target MCU Configuration
# =============================================================================

set(MCU_FAMILY STM32F7xx)
set(MCU_DEVICE STM32F722xx)
set(MCU_FLASH_SIZE 512K)
set(MCU_RAM_SIZE 256K)

# =============================================================================
# HAL/CMSIS Configuration
# =============================================================================

# Prefer environment variable (set by Nix flake), fall back to local path
if(DEFINED ENV{STM32CUBE_PATH})
    set(STM32CUBE_PATH "$ENV{STM32CUBE_PATH}")
else()
    set(STM32CUBE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib/STM32CubeF7)
endif()

# Check if HAL exists
if(NOT EXISTS "${STM32CUBE_PATH}/Drivers/STM32F7xx_HAL_Driver/Inc")
    message(WARNING "
=============================================================================
STM32CubeF7 HAL not found at: ${STM32CUBE_PATH}

If using Nix: run 'nix develop' to enter the dev shell (HAL provided via flake)
Otherwise:   git clone --recursive https://github.com/STMicroelectronics/STM32CubeF7.git lib/STM32CubeF7

Building with stubs for now...
=============================================================================
")
    set(USE_HAL_STUBS ON)
else()
    message(STATUS "Using STM32CubeF7 from: ${STM32CUBE_PATH}")
    set(USE_HAL_STUBS OFF)
endif()

# =============================================================================
# Source Files
# =============================================================================

set(SOURCES
    # Main
    src/main.c

    # Drivers
    src/drivers/icm42688.c
    src/drivers/crsf.c
    src/drivers/esc.c
    src/drivers/srxl2.c
    src/drivers/pwm.c
    src/drivers/uart.c
    src/drivers/spi.c

    # USB CDC
    src/usb/usb_cdc.c
    src/usb/usbd_conf.c
    src/usb/usbd_desc.c
    src/usb/usbd_cdc_if.c

    # Flight control
    src/flight/gyro.c
    src/flight/speed.c
    src/flight/mixer.c
    src/flight/stabilizer.c

    # Configuration
    src/config/config.c
    src/config/cli.c
    src/config/eeprom.c

    # Target-specific
    src/target/nexus/target.c
)

set(STARTUP_SOURCES
    startup/startup_stm32f722xx.s
)

# USB Device Library sources (from STM32CubeF7)
if(NOT USE_HAL_STUBS)
    list(APPEND SOURCES
        # USB Middleware
        ${STM32CUBE_PATH}/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c
        ${STM32CUBE_PATH}/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c
        ${STM32CUBE_PATH}/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c
        ${STM32CUBE_PATH}/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.c
        
        # HAL Drivers we use
        ${STM32CUBE_PATH}/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal.c
        ${STM32CUBE_PATH}/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_cortex.c
        ${STM32CUBE_PATH}/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_rcc.c
        ${STM32CUBE_PATH}/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_rcc_ex.c
        ${STM32CUBE_PATH}/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_gpio.c
        ${STM32CUBE_PATH}/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_pwr.c
        ${STM32CUBE_PATH}/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_pwr_ex.c
        ${STM32CUBE_PATH}/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_pcd.c
        ${STM32CUBE_PATH}/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_pcd_ex.c
        ${STM32CUBE_PATH}/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_spi.c
        ${STM32CUBE_PATH}/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_tim.c
        ${STM32CUBE_PATH}/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_tim_ex.c
        ${STM32CUBE_PATH}/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_uart.c
        ${STM32CUBE_PATH}/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_uart_ex.c
        ${STM32CUBE_PATH}/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_iwdg.c
        ${STM32CUBE_PATH}/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_ll_usb.c
    )
endif()

# =============================================================================
# Executable
# =============================================================================

add_executable(${PROJECT_NAME}
    ${SOURCES}
    ${STARTUP_SOURCES}
)

# =============================================================================
# Include Directories
# =============================================================================

target_include_directories(${PROJECT_NAME} PRIVATE
    src
    src/drivers
    src/flight
    src/config
    src/target/nexus
    src/usb
)

if(USE_HAL_STUBS)
    target_include_directories(${PROJECT_NAME} PRIVATE
        lib/stubs
    )
    target_compile_definitions(${PROJECT_NAME} PRIVATE
        USE_HAL_STUBS
    )
else()
    # STM32CubeF7 HAL includes
    target_include_directories(${PROJECT_NAME} PRIVATE
        ${STM32CUBE_PATH}/Drivers/STM32F7xx_HAL_Driver/Inc
        ${STM32CUBE_PATH}/Drivers/STM32F7xx_HAL_Driver/Inc/Legacy
        ${STM32CUBE_PATH}/Drivers/CMSIS/Device/ST/STM32F7xx/Include
        ${STM32CUBE_PATH}/Drivers/CMSIS/Include
        ${STM32CUBE_PATH}/Middlewares/ST/STM32_USB_Device_Library/Core/Inc
        ${STM32CUBE_PATH}/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc
    )
endif()

# =============================================================================
# Compile Definitions
# =============================================================================

target_compile_definitions(${PROJECT_NAME} PRIVATE
    ${MCU_DEVICE}
    USE_HAL_DRIVER
    HSE_VALUE=8000000U  # 8MHz external crystal on Nexus
)

# =============================================================================
# Linker Script
# =============================================================================

set(LINKER_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/linker/STM32F722RETx_FLASH.ld)

target_link_options(${PROJECT_NAME} PRIVATE
    -T${LINKER_SCRIPT}
    -Wl,--print-memory-usage
)

# Ensure linker script changes trigger re-link
set_target_properties(${PROJECT_NAME} PROPERTIES
    LINK_DEPENDS ${LINKER_SCRIPT}
)

# =============================================================================
# Warnings
# =============================================================================

target_compile_options(${PROJECT_NAME} PRIVATE
    -Wall
    -Wextra
    -Wno-unused-parameter  # HAL has many unused params
    -Werror=return-type
)

# =============================================================================
# Post-build: Generate .bin and .hex, print size
# =============================================================================

add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${PROJECT_NAME}> ${PROJECT_NAME}.bin
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${PROJECT_NAME}> ${PROJECT_NAME}.hex
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${PROJECT_NAME}>
    COMMENT "Building ${PROJECT_NAME}.bin and ${PROJECT_NAME}.hex"
)
