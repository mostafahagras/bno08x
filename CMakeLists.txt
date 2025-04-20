cmake_minimum_required(VERSION 3.13)

# Allow library to work standalone or as part of another project
if (NOT DEFINED PICO_SDK_PATH)
    # If not already set, use the environment variable
    set(PICO_SDK_PATH $ENV{PICO_SDK_PATH})
endif()

# Import the Pico SDK
include(pico_sdk_import.cmake)

project(pico C CXX ASM)

# Initialize the SDK (only if building standalone)
if (NOT TARGET pico_stdlib)
    pico_sdk_init()
endif()

add_subdirectory(lib/sh2)

# Define the library
add_library(bno08x STATIC
    src/bno08x.c
)

# Include headers (public for users)
target_include_directories(bno08x
    PUBLIC 
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Link SDK dependencies
target_link_libraries(bno08x
    PRIVATE
        pico_stdlib
        hardware_i2c
        hardware_spi
        hardware_uart
        sh2
)