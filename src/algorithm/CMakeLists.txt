# Set the source files for the algorithms
set(ALGORITHM_SOURCES
    sensorless_trapezoidal.c
    foc.c
    pid.c
    # add more algorithm files here as needed
)

# Add the source files to the library
add_library(algorithms STATIC ${ALGORITHM_SOURCES})

# Include the necessary header files for the algorithms
target_include_directories(algorithms PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_include_directories(algorithms PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/../mcu)

# Link the algorithms library to the motor controller executable
target_link_libraries(motor_controller PRIVATE algorithms)
