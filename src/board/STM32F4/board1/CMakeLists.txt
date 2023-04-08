# Set the BOARD option
set(BOARD "board1" CACHE STRING "Target board")

# Add the board-specific header file
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../mcu/STM32F4/${BOARD})

# Add the source files for the board-specific peripherals
set(SOURCE_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/gpio.c
    ${CMAKE_CURRENT_SOURCE_DIR}/adc.c
    ${CMAKE_CURRENT_SOURCE_DIR}/uart.c
    ${CMAKE_CURRENT_SOURCE_DIR}/timer.c
    ${CMAKE_CURRENT_SOURCE_DIR}/pwm.c
    # Add any other required source files here
)

add_library(board STATIC ${SOURCE_FILES})