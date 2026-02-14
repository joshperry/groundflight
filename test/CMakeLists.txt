# GroundFlight Host-Side Tests
#
# Builds with host gcc (not arm-none-eabi). Tests pure-logic modules
# that have zero HAL dependencies, plus integration tests using mocks.

cmake_minimum_required(VERSION 3.20)

project(groundflight-tests
    VERSION 0.1.0
    LANGUAGES C
)

# Use host compiler, not ARM cross
set(CMAKE_C_STANDARD 99)

# =============================================================================
# Source paths
# =============================================================================

set(SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../src)
set(MOCK_DIR ${CMAKE_CURRENT_SOURCE_DIR}/mocks)
set(UNITY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/unity)

# =============================================================================
# Unity test framework
# =============================================================================

add_library(unity STATIC ${UNITY_DIR}/unity.c)
target_include_directories(unity PUBLIC ${UNITY_DIR})

# =============================================================================
# Mock libraries
# =============================================================================

add_library(mocks STATIC
    ${MOCK_DIR}/mock_uart.c
    ${MOCK_DIR}/mock_pwm.c
    ${MOCK_DIR}/mock_eeprom.c
)
target_include_directories(mocks PUBLIC
    ${MOCK_DIR}
    ${SRC_DIR}
    ${SRC_DIR}/drivers
    ${SRC_DIR}/flight
    ${SRC_DIR}/config
)
target_compile_definitions(mocks PUBLIC
    GROUNDFLIGHT_TEST
)

# =============================================================================
# Common includes for all tests
# =============================================================================

set(TEST_INCLUDES
    ${SRC_DIR}
    ${SRC_DIR}/drivers
    ${SRC_DIR}/flight
    ${SRC_DIR}/config
    ${MOCK_DIR}
    ${UNITY_DIR}
)

set(TEST_DEFINES
    GROUNDFLIGHT_TEST
)

# =============================================================================
# Warnings
# =============================================================================

add_compile_options(
    -Wall
    -Wextra
    -Wno-unused-parameter
    -Werror=return-type
)

# =============================================================================
# Helper: add a test executable
# =============================================================================

function(add_gf_test TEST_NAME)
    # Remaining args are source files
    set(SOURCES ${ARGN})
    add_executable(${TEST_NAME} ${SOURCES})
    target_include_directories(${TEST_NAME} PRIVATE ${TEST_INCLUDES})
    target_compile_definitions(${TEST_NAME} PRIVATE ${TEST_DEFINES})
    target_link_libraries(${TEST_NAME} PRIVATE unity mocks m)
    add_test(NAME ${TEST_NAME} COMMAND ${TEST_NAME})
endfunction()

# =============================================================================
# Unit tests
# =============================================================================

# Stabilizer: pure math, no HAL
add_gf_test(test_stabilizer
    unit/test_stabilizer.c
    ${SRC_DIR}/flight/stabilizer.c
)

# Mixer: pure math, no HAL
add_gf_test(test_mixer
    unit/test_mixer.c
    ${SRC_DIR}/flight/mixer.c
)

# Speed: pure math, no HAL
add_gf_test(test_speed
    unit/test_speed.c
    ${SRC_DIR}/flight/speed.c
)

# Gyro: pure math (biquad filter), no HAL
add_gf_test(test_gyro
    unit/test_gyro.c
    ${SRC_DIR}/flight/gyro.c
)

# Config: pure data, no HAL
add_gf_test(test_config
    unit/test_config.c
    ${SRC_DIR}/config/config.c
)

# CRSF conversion functions: inline in header, no HAL needed
add_gf_test(test_crsf
    unit/test_crsf.c
)

# MSP frame encoding: uses mock UART
add_gf_test(test_msp
    unit/test_msp.c
    ${SRC_DIR}/drivers/msp.c
)

# =============================================================================
# Integration tests
# =============================================================================

# Full signal chain: CRSF -> stabilizer -> mixer -> PWM
add_gf_test(test_signal_chain
    integration/test_signal_chain.c
    ${SRC_DIR}/flight/stabilizer.c
    ${SRC_DIR}/flight/mixer.c
    ${SRC_DIR}/flight/gyro.c
    ${SRC_DIR}/flight/speed.c
    ${SRC_DIR}/config/config.c
)

# Failsafe: verify safe outputs on link loss
add_gf_test(test_failsafe
    integration/test_failsafe.c
    ${SRC_DIR}/flight/stabilizer.c
    ${SRC_DIR}/flight/mixer.c
    ${SRC_DIR}/config/config.c
)

# =============================================================================
# Enable CTest
# =============================================================================

enable_testing()
