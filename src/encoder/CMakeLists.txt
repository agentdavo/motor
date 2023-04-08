# Add your source files here
set(SOURCE_FILES
    encoder.c
    biss_encoder.c
    resolver_encoder.c
    # ... (other encoder source files)
)

# Include the board folder to handle board-specific header files
add_subdirectory(../board board)

# Include the MCU-specific folder to handle MCU-specific header files
add_subdirectory(../mcu mcu)

# Add the encoder-specific header files
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Add the encoder library target
add_library(encoder STATIC ${SOURCE_FILES})

# Link the encoder library with the board and MCU libraries
target_link_libraries(encoder board mcu)