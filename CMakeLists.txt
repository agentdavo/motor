cmake_minimum_required(VERSION 3.10)

project(motor_controller C)

set(CMAKE_C_STANDARD 99)

# Define the MCU type (default to RZT2M)
set(MCU_TYPE "RZT2M" CACHE STRING "Selected MCU type")

# Add your source files here
set(SOURCE_FILES
    src/main.c
    # ... (other source files)
)

# Add the MCU-specific source files
include_directories(src/mcu)
file(GLOB MCU_SOURCES src/mcu/*.c)
list(APPEND SOURCE_FILES ${MCU_SOURCES})

# Include the board-specific source files
include_directories(src/board/${MCU_TYPE})
file(GLOB BOARD_SOURCES src/board/${MCU_TYPE}/*.c)
list(APPEND SOURCE_FILES ${BOARD_SOURCES})

# Add the encoder-specific source files
include_directories(src/encoder)
file(GLOB ENCODER_SOURCES src/encoder/*.c)
list(APPEND SOURCE_FILES ${ENCODER_SOURCES})

# Include the appropriate FSP headers based on the selected MCU
if(MCU_TYPE STREQUAL "RZT2M")
    target_compile_definitions(motor_controller PRIVATE -DMCU_RZT2M)
    target_include_directories(motor_controller PRIVATE ${PROJECT_SOURCE_DIR}/src/mcu/RZT2M/fsp)
elseif(MCU_TYPE STREQUAL "RZA")
    target_compile_definitions(motor_controller PRIVATE -DMCU_RZA)
    target_include_directories(motor_controller PRIVATE ${PROJECT_SOURCE_DIR}/src/mcu/RZA/fsp)
else()
    message(FATAL_ERROR "Unsupported MCU type: ${MCU_TYPE}")
endif()

add_executable(motor_controller ${SOURCE_FILES})