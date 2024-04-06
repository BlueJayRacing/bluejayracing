# Minimum required CMake version
cmake_minimum_required(VERSION 3.28)
project(baja_ipc VERSION 1.0.0)

# get all of the source and headers by globbing through the src directory
if (NOT TARGET baja_ipc)
file(GLOB_RECURSE SOURCES "src/*.cpp")
file(GLOB_RECURSE HEADERS "include/**/*.h")
add_library(baja_ipc STATIC ${SOURCES} ${HEADERS})
target_include_directories(baja_ipc PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_BINARY_DIR}
)

# Recursively link any dependencies
add_subdirectory(
    ${CMAKE_CURRENT_SOURCE_DIR}/../baja_proto
    ${CMAKE_CURRENT_SOURCE_DIR}/../baja_proto/bin_for_ipc
)
target_link_libraries(baja_ipc
    PUBLIC baja_proto
    # Add any required libraries here
)
endif()
