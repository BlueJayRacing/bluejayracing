# Minimum required CMake version
cmake_minimum_required(VERSION 3.28)
project(baja_rtk VERSION 1.0.0)

# get all of the source and headers by globbing through the src directory

if (NOT TARGET baja_rtk)
    file(GLOB_RECURSE SOURCES "src/*.cpp")
    file(GLOB_RECURSE HEADERS "include/**/*.h")
    add_library(baja_rtk STATIC ${SOURCES} ${HEADERS})
    target_include_directories(baja_rtk PUBLIC 
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_BINARY_DIR}/proto
        ${CMAKE_CURRENT_BINARY_DIR}
    )
endif()