cmake_minimum_required(VERSION 3.21)

# Set the cross toolchain before project() for both presets and direct use.
if(NOT DEFINED CMAKE_TOOLCHAIN_FILE)
    set(CMAKE_TOOLCHAIN_FILE "${CMAKE_CURRENT_SOURCE_DIR}/cmake/arm-none-eabi.cmake"
        CACHE FILEPATH "Arm GNU toolchain")
endif()
project(open_imu LANGUAGES C ASM)

if(NOT CMAKE_C_COMPILER_ID STREQUAL "GNU")
    message(FATAL_ERROR "This firmware build requires Arm GNU GCC")
endif()
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug CACHE STRING "Debug or Release" FORCE)
endif()
if(NOT CMAKE_BUILD_TYPE MATCHES "^(Debug|Release)$")
    message(FATAL_ERROR "Choose Debug or Release to match the CubeIDE configurations")
endif()

# Match .cproject rather than CMake's host defaults (-O3/-DNDEBUG in Release).
set(CMAKE_C_FLAGS_DEBUG "-O0 -g3" CACHE STRING "CubeIDE Debug C flags" FORCE)
set(CMAKE_C_FLAGS_RELEASE "-Os -g0" CACHE STRING "CubeIDE Release C flags" FORCE)
set(CMAKE_ASM_FLAGS_DEBUG "-g3" CACHE STRING "CubeIDE Debug assembly flags" FORCE)
set(CMAKE_ASM_FLAGS_RELEASE "-g0" CACHE STRING "CubeIDE Release assembly flags" FORCE)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(OPENIMU_SOURCE_TIMESTAMP "Enable the optional OpenIMU source timestamp/FIFO path" OFF)
include(cmake/sources.cmake)

add_executable(open_imu ${OPENIMU_BASE_SOURCES})
set_target_properties(open_imu PROPERTIES SUFFIX ".elf" C_STANDARD 11
    C_STANDARD_REQUIRED YES C_EXTENSIONS YES)
if(OPENIMU_SOURCE_TIMESTAMP)
    foreach(source Core/Src/imu_timestamp.c Core/Src/icm45686_fifo.c)
        if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${source}")
            message(FATAL_ERROR "OPENIMU_SOURCE_TIMESTAMP requires ${source}")
        endif()
        target_sources(open_imu PRIVATE "${source}")
    endforeach()
endif()

target_include_directories(open_imu PRIVATE
    Core/Inc
    Drivers/STM32G4xx_HAL_Driver/Inc
    Drivers/STM32G4xx_HAL_Driver/Inc/Legacy
    Middlewares/Third_Party/FreeRTOS/Source/include
    Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2
    Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F
    Drivers/CMSIS/Device/ST/STM32G4xx/Include
    Drivers/CMSIS/Include
    "$<$<COMPILE_LANGUAGE:C>:${CMAKE_CURRENT_SOURCE_DIR}/Core/ThreadSafe>")
target_compile_definitions(open_imu PRIVATE
    "$<$<CONFIG:Debug>:DEBUG>"
    "$<$<COMPILE_LANGUAGE:C>:USE_HAL_DRIVER>"
    "$<$<COMPILE_LANGUAGE:C>:STM32G431xx>"
    "$<$<COMPILE_LANGUAGE:C>:STM32_THREAD_SAFE_STRATEGY=4>"
    "$<$<COMPILE_LANGUAGE:C>:IMU_ENABLE_SOURCE_TIMESTAMP=$<BOOL:${OPENIMU_SOURCE_TIMESTAMP}>>")
set(OPENIMU_CPU_FLAGS -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard)
target_compile_options(open_imu PRIVATE ${OPENIMU_CPU_FLAGS}
    "$<$<COMPILE_LANGUAGE:C>:--specs=nano.specs>"
    "$<$<COMPILE_LANGUAGE:C>:-ffunction-sections>"
    "$<$<COMPILE_LANGUAGE:C>:-fdata-sections>"
    "$<$<COMPILE_LANGUAGE:C>:-Wall>"
    "$<$<COMPILE_LANGUAGE:C>:-fstack-usage>")
set_source_files_properties(Core/Startup/startup_stm32g431cbux.s
    PROPERTIES COMPILE_OPTIONS "-x;assembler-with-cpp")

set(OPENIMU_LINKER_SCRIPT "${CMAKE_CURRENT_SOURCE_DIR}/STM32G431CBUX_FLASH.ld")
set_property(TARGET open_imu APPEND PROPERTY LINK_DEPENDS "${OPENIMU_LINKER_SCRIPT}")
target_link_options(open_imu PRIVATE ${OPENIMU_CPU_FLAGS}
    "-T${OPENIMU_LINKER_SCRIPT}" --specs=nano.specs --specs=nosys.specs
    -static -Wl,--gc-sections -Wl,--print-memory-usage
    "-Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/open_imu.map")
target_link_libraries(open_imu PRIVATE -Wl,--start-group c m -Wl,--end-group)
add_custom_command(TARGET open_imu POST_BUILD
    COMMAND "${CMAKE_OBJCOPY}" -O binary "$<TARGET_FILE:open_imu>" "${CMAKE_CURRENT_BINARY_DIR}/open_imu.bin"
    COMMAND "${CMAKE_OBJCOPY}" -O ihex "$<TARGET_FILE:open_imu>" "${CMAKE_CURRENT_BINARY_DIR}/open_imu.hex"
    COMMAND "${CMAKE_SIZE}" "$<TARGET_FILE:open_imu>"
    BYPRODUCTS "${CMAKE_CURRENT_BINARY_DIR}/open_imu.bin" "${CMAKE_CURRENT_BINARY_DIR}/open_imu.hex"
    COMMENT "Generate OpenIMU BIN/HEX and report firmware size" VERBATIM)
message(STATUS "OpenIMU source timestamps: ${OPENIMU_SOURCE_TIMESTAMP}")
message(STATUS "Arm compiler: ${CMAKE_C_COMPILER} (${CMAKE_C_COMPILER_VERSION})")
