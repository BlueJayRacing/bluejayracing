# Minimum required CMake version
cmake_minimum_required(VERSION 3.28)
project(comms VERSION 1.0.0)

# get all of the source and headers by globbing through the src directory

file(GLOB_RECURSE SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")
file(GLOB_RECURSE HEADERS "${CMAKE_CURRENT_SOURCE_DIR}/include/**/*.h")

add_library(comms STATIC ${SOURCES} ${HEADERS})

target_include_directories(comms PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_BINARY_DIR}
)
