set(CMAKE_SYSTEM_NAME               Generic)
set(CMAKE_SYSTEM_PROCESSOR          arm)

set(CMAKE_C_COMPILER_ID GNU)
set(CMAKE_CXX_COMPILER_ID GNU)

# Some default GCC settings.
# You can force a specific toolchain by setting GCC_TOOLCHAIN_BIN_DIR to the
# folder that contains arm-none-eabi-gcc(.exe).
set(GCC_TOOLCHAIN_BIN_DIR "" CACHE PATH "Path to ARM GCC 'bin' directory")

if(WIN32 AND NOT GCC_TOOLCHAIN_BIN_DIR)
    # Auto-detect STM32CubeIDE bundled GCC on Windows.
    file(GLOB _STM32_GCC_BIN_DIRS
        "C:/ST/STM32CubeIDE_*/STM32CubeIDE/plugins/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32*/tools/bin"
    )
    list(SORT _STM32_GCC_BIN_DIRS COMPARE NATURAL ORDER DESCENDING)
    list(LENGTH _STM32_GCC_BIN_DIRS _STM32_GCC_BIN_DIRS_LEN)
    if(_STM32_GCC_BIN_DIRS_LEN GREATER 0)
        list(GET _STM32_GCC_BIN_DIRS 0 GCC_TOOLCHAIN_BIN_DIR)
        message(STATUS "Using STM32CubeIDE toolchain at: ${GCC_TOOLCHAIN_BIN_DIR}")
    endif()
endif()

if(GCC_TOOLCHAIN_BIN_DIR)
    file(TO_CMAKE_PATH "${GCC_TOOLCHAIN_BIN_DIR}" GCC_TOOLCHAIN_BIN_DIR)
    set(TOOLCHAIN_PREFIX "${GCC_TOOLCHAIN_BIN_DIR}/arm-none-eabi-")
else()
    # Fallback to PATH lookup.
    set(TOOLCHAIN_PREFIX arm-none-eabi-)
endif()

if(WIN32)
    set(TOOL_EXECUTABLE_SUFFIX ".exe")
else()
    set(TOOL_EXECUTABLE_SUFFIX "")
endif()

set(CMAKE_C_COMPILER                ${TOOLCHAIN_PREFIX}gcc${TOOL_EXECUTABLE_SUFFIX})
set(CMAKE_ASM_COMPILER              ${CMAKE_C_COMPILER})
set(CMAKE_CXX_COMPILER              ${TOOLCHAIN_PREFIX}g++${TOOL_EXECUTABLE_SUFFIX})
set(CMAKE_LINKER                    ${TOOLCHAIN_PREFIX}g++${TOOL_EXECUTABLE_SUFFIX})
set(CMAKE_OBJCOPY                   ${TOOLCHAIN_PREFIX}objcopy${TOOL_EXECUTABLE_SUFFIX})
set(CMAKE_SIZE                      ${TOOLCHAIN_PREFIX}size${TOOL_EXECUTABLE_SUFFIX})

set(CMAKE_EXECUTABLE_SUFFIX_ASM     ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_C       ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_CXX     ".elf")

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# MCU specific flags
set(TARGET_FLAGS "-mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard ")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${TARGET_FLAGS}")
set(CMAKE_ASM_FLAGS "${CMAKE_C_FLAGS} -x assembler-with-cpp -MMD -MP")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -fdata-sections -ffunction-sections -fstack-usage")

# The cyclomatic-complexity parameter must be defined for the Cyclomatic complexity feature in STM32CubeIDE to work.
# However, most GCC toolchains do not support this option, which causes a compilation error; for this reason, the feature is disabled by default.
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fcyclomatic-complexity")

set(CMAKE_C_FLAGS_DEBUG "-O0 -g3")
set(CMAKE_C_FLAGS_RELEASE "-Os -g0")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g3")
set(CMAKE_CXX_FLAGS_RELEASE "-Os -g0")

set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -fno-rtti -fno-exceptions -fno-threadsafe-statics")

set(CMAKE_EXE_LINKER_FLAGS "${TARGET_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -T \"${CMAKE_SOURCE_DIR}/STM32L476XX_FLASH.ld\"")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --specs=nano.specs")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-Map=${CMAKE_PROJECT_NAME}.map -Wl,--gc-sections")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--print-memory-usage")
set(TOOLCHAIN_LINK_LIBRARIES "m")
