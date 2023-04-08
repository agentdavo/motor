# Set the MCU and BOARD options
set(MCU "RZT2M" CACHE STRING "Target MCU")
set(BOARD "board1" CACHE STRING "Target board")

# Include the MCU-specific header files
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../mcu/${MCU}/include)

# Add the board-specific header files
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/${MCU}/${BOARD}/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/${MCU}/${BOARD}/src)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/${MCU}/${BOARD}/src/adc)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/${MCU}/${BOARD}/src/dma)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/${MCU}/${BOARD}/src/gpio)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/${MCU}/${BOARD}/src/timers)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/${MCU}/${BOARD}/src/uart)

# Add the board-specific source files
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/${MCU}/${BOARD}/src)
