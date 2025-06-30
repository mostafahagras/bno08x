cmake_minimum_required(VERSION 3.13)

project(bno08x C)

add_library(${PROJECT_NAME} INTERFACE)

target_sources(${PROJECT_NAME} INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/src/bno08x.c
)

target_include_directories(${PROJECT_NAME} INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/include
)

target_link_libraries(${PROJECT_NAME} INTERFACE
    pico_stdlib
    hardware_i2c
    hardware_gpio
    hardware_timer
)

if(EXISTS ${CMAKE_CURRENT_LIST_DIR}/lib/sh2)
    add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/lib/sh2)
    target_link_libraries(${PROJECT_NAME} INTERFACE sh2)
else()
    message(FATAL_ERROR "SH2 library not found. Please ensure the Hillcrest SH2 library is available.")
endif()

if(EXISTS ${CMAKE_CURRENT_LIST_DIR}/lib/sh2/include)
    target_include_directories(${PROJECT_NAME} INTERFACE
        ${CMAKE_CURRENT_LIST_DIR}/lib/sh2/include
    )
endif()

