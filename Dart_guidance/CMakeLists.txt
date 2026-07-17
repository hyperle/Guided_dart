cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

add_compile_options(
    -mcpu=cortex-m4
    -mthumb
    -mfloat-abi=hard
    -mfpu=fpv4-sp-d16
)

add_definitions(
    -DARM_MATH_CM4
    -DARM_MATH_DSP
    -DUSE_HAL_DRIVER
    -DSTM32G431xx
)

add_link_options(
    -mfloat-abi=hard
    -mfpu=fpv4-sp-d16
)

# Set the project name
set(CMAKE_PROJECT_NAME STM32)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

find_package(Python3 COMPONENTS Interpreter REQUIRED)

add_custom_command(
    OUTPUT ${CMAKE_SOURCE_DIR}/Drv/Inc/guidance_protocol_generated.h
    COMMAND ${Python3_EXECUTABLE}
            ${CMAKE_SOURCE_DIR}/../protocol/guidance_protocol.py
            --schema ${CMAKE_SOURCE_DIR}/../protocol/guidance_protocol.yaml
            --header ${CMAKE_SOURCE_DIR}/Drv/Inc/guidance_protocol_generated.h
    DEPENDS
        ${CMAKE_SOURCE_DIR}/../protocol/guidance_protocol.py
        ${CMAKE_SOURCE_DIR}/../protocol/guidance_protocol.yaml
    COMMENT "Generating shared guidance protocol header"
    VERBATIM
)

add_custom_target(guidance_protocol_header
    DEPENDS ${CMAKE_SOURCE_DIR}/Drv/Inc/guidance_protocol_generated.h
)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})
add_dependencies(${CMAKE_PROJECT_NAME} guidance_protocol_header)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    Drv/Src/bmi088.c
    Drv/Src/esp32_link.c
    Drv/Src/servo.c
    Drv/Src/interface.c
    Lib/Src/mahony.c
    Lib/Src/pid.c
    APP/Src/imu.c
    APP/Src/task.c
    APP/Src/target_smoother.c
    APP/Src/green_light_task.c
    APP/Src/pixel_delta_pid_action.c
    APP/Src/imu_velocity_recovery_action.c
    APP/Src/timed_turn_pulse_action.c
    APP/Src/control_mixer.c
    APP/Src/guidance_controller.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    Drivers/CMSIS/DSP/Include
    Drv/Inc
    Lib/Inc
    APP/Inc
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/DSP/Lib/GCC/libarm_cortexM4lf_math.a
    # Add user defined libraries
)
