# ARM Cortex-M7 toolchain file for STM32F722
# GroundFlight - RadioMaster Nexus target

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

# Toolchain paths (Nix sets ARM_TOOLCHAIN_PATH)
set(TOOLCHAIN_PREFIX "arm-none-eabi-")

# Use the toolchain from environment or default location
if(DEFINED ENV{ARM_TOOLCHAIN_PATH})
    set(TOOLCHAIN_PATH "$ENV{ARM_TOOLCHAIN_PATH}/")
else()
    set(TOOLCHAIN_PATH "")
endif()

set(CMAKE_C_COMPILER   "${TOOLCHAIN_PATH}${TOOLCHAIN_PREFIX}gcc")
set(CMAKE_CXX_COMPILER "${TOOLCHAIN_PATH}${TOOLCHAIN_PREFIX}g++")
set(CMAKE_ASM_COMPILER "${TOOLCHAIN_PATH}${TOOLCHAIN_PREFIX}gcc")
set(CMAKE_OBJCOPY      "${TOOLCHAIN_PATH}${TOOLCHAIN_PREFIX}objcopy")
set(CMAKE_OBJDUMP      "${TOOLCHAIN_PATH}${TOOLCHAIN_PREFIX}objdump")
set(CMAKE_SIZE         "${TOOLCHAIN_PATH}${TOOLCHAIN_PREFIX}size")

# STM32F722 - Cortex-M7 with FPU
set(CPU_FLAGS "-mcpu=cortex-m7 -mthumb -mfloat-abi=hard -mfpu=fpv5-sp-d16")

# Common flags
set(COMMON_FLAGS "${CPU_FLAGS} -ffunction-sections -fdata-sections -fno-common -fmessage-length=0")

# C flags
set(CMAKE_C_FLAGS_INIT "${COMMON_FLAGS}")
set(CMAKE_C_FLAGS_DEBUG "-Og -g3 -DDEBUG")
set(CMAKE_C_FLAGS_RELEASE "-O2 -DNDEBUG")

# C++ flags (if needed later)
set(CMAKE_CXX_FLAGS_INIT "${COMMON_FLAGS} -fno-rtti -fno-exceptions")

# ASM flags
set(CMAKE_ASM_FLAGS_INIT "${CPU_FLAGS} -x assembler-with-cpp")

# Linker flags
set(CMAKE_EXE_LINKER_FLAGS_INIT "${CPU_FLAGS} -specs=nosys.specs -specs=nano.specs -Wl,--gc-sections -Wl,-Map=groundflight.map")

# Don't try to run test executables on host
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# Search paths for find_* commands
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
