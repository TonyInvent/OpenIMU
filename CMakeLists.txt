cmake_minimum_required(VERSION 3.21)
# Optional command-line build of the existing STM32CubeIDE firmware.
set(CMAKE_TOOLCHAIN_FILE "${CMAKE_CURRENT_LIST_DIR}/cmake/arm-none-eabi.cmake" CACHE FILEPATH "Arm toolchain")
project(open_imu LANGUAGES C ASM)
set(OPENIMU_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
option(OPENIMU_CAN_SYNC "Enable experimental, host-activated CAN clock synchronization" OFF)
set(OPENIMU_NODE_ID "1" CACHE STRING "Unique CAN node ID (1..5)")
if(NOT OPENIMU_NODE_ID MATCHES "^[1-5]$")
    message(FATAL_ERROR "OPENIMU_NODE_ID must be 1..5")
endif()
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Debug or Release" FORCE)
endif()
if(NOT CMAKE_BUILD_TYPE MATCHES "^(Debug|Release)$")
    message(FATAL_ERROR "Use Debug or Release to match upstream .cproject")
endif()
# Match upstream CubeIDE flags rather than CMake's -O3/-DNDEBUG defaults.
set(CMAKE_C_FLAGS_DEBUG "-O0 -g3" CACHE STRING "CubeIDE Debug flags" FORCE)
set(CMAKE_C_FLAGS_RELEASE "-Os -g0" CACHE STRING "CubeIDE Release flags" FORCE)
set(CMAKE_ASM_FLAGS_DEBUG "-g3" CACHE STRING "Debug assembly flags" FORCE)
set(CMAKE_ASM_FLAGS_RELEASE "-g0" CACHE STRING "Release assembly flags" FORCE)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
file(GLOB core CONFIGURE_DEPENDS "${OPENIMU_SOURCE_DIR}/Core/Src/*.c" "${OPENIMU_SOURCE_DIR}/Core/ThreadSafe/*.c")
file(GLOB hal CONFIGURE_DEPENDS "${OPENIMU_SOURCE_DIR}/Drivers/STM32G4xx_HAL_Driver/Src/*.c")
set(rtos "${OPENIMU_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/Source")
file(GLOB kernel CONFIGURE_DEPENDS "${rtos}/*.c")
add_executable(open_imu ${core} ${hal} ${kernel}
    "${OPENIMU_SOURCE_DIR}/Core/Startup/startup_stm32g431cbux.s"
    "${rtos}/CMSIS_RTOS_V2/cmsis_os2.c"
    "${rtos}/portable/GCC/ARM_CM4F/port.c"
    "${rtos}/portable/MemMang/heap_4.c")
set_target_properties(open_imu PROPERTIES SUFFIX ".elf" C_STANDARD 11 C_STANDARD_REQUIRED YES C_EXTENSIONS YES)
foreach(dir Core/Inc Drivers/STM32G4xx_HAL_Driver/Inc Drivers/STM32G4xx_HAL_Driver/Inc/Legacy
            Drivers/CMSIS/Device/ST/STM32G4xx/Include Drivers/CMSIS/Include Core/ThreadSafe)
    target_include_directories(open_imu PRIVATE "${OPENIMU_SOURCE_DIR}/${dir}")
endforeach()
target_include_directories(open_imu PRIVATE "${rtos}/include" "${rtos}/CMSIS_RTOS_V2" "${rtos}/portable/GCC/ARM_CM4F")
target_compile_definitions(open_imu PRIVATE "$<$<CONFIG:Debug>:DEBUG>"
    "$<$<COMPILE_LANGUAGE:C>:USE_HAL_DRIVER>" "$<$<COMPILE_LANGUAGE:C>:STM32G431xx>"
    "$<$<COMPILE_LANGUAGE:C>:STM32_THREAD_SAFE_STRATEGY=4>")
target_compile_definitions(open_imu PRIVATE OPENIMU_CAN_SYNC=$<BOOL:${OPENIMU_CAN_SYNC}> OPENIMU_NODE_ID=${OPENIMU_NODE_ID})
set(cpu -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard)
target_compile_options(open_imu PRIVATE ${cpu} "$<$<COMPILE_LANGUAGE:C>:--specs=nano.specs>"
    "$<$<COMPILE_LANGUAGE:C>:-ffunction-sections>" "$<$<COMPILE_LANGUAGE:C>:-fdata-sections>"
    "$<$<COMPILE_LANGUAGE:C>:-Wall>" "$<$<COMPILE_LANGUAGE:C>:-fstack-usage>")
set_source_files_properties("${OPENIMU_SOURCE_DIR}/Core/Startup/startup_stm32g431cbux.s"
    PROPERTIES COMPILE_OPTIONS "-x;assembler-with-cpp")
set(linker "${OPENIMU_SOURCE_DIR}/STM32G431CBUX_FLASH.ld")
set_property(TARGET open_imu APPEND PROPERTY LINK_DEPENDS "${linker}")
target_link_options(open_imu PRIVATE ${cpu} "-T${linker}" --specs=nano.specs --specs=nosys.specs
    -static -Wl,--gc-sections -Wl,--print-memory-usage "-Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/open_imu.map")
target_link_libraries(open_imu PRIVATE -Wl,--start-group c m -Wl,--end-group)
add_custom_command(TARGET open_imu POST_BUILD
    COMMAND "${CMAKE_OBJCOPY}" -O binary "$<TARGET_FILE:open_imu>" "${CMAKE_CURRENT_BINARY_DIR}/open_imu.bin"
    COMMAND "${CMAKE_OBJCOPY}" -O ihex "$<TARGET_FILE:open_imu>" "${CMAKE_CURRENT_BINARY_DIR}/open_imu.hex"
    COMMAND "${CMAKE_SIZE}" "$<TARGET_FILE:open_imu>"
    BYPRODUCTS "${CMAKE_CURRENT_BINARY_DIR}/open_imu.bin" "${CMAKE_CURRENT_BINARY_DIR}/open_imu.hex")
