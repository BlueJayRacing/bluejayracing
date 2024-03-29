# Minimum required CMake version
cmake_minimum_required(VERSION 3.28)
project(rtk_baja VERSION 1.0.0)

# get all of the source and headers by globbing through the src directory
file(GLOB_RECURSE SOURCES "src/*.cpp")
file(GLOB_RECURSE HEADERS "include/**/*.h")
add_library(rtk_baja STATIC ${SOURCES} ${HEADERS})
target_include_directories(rtk_baja PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_BINARY_DIR}/proto
    ${CMAKE_CURRENT_BINARY_DIR}
)

# Recursively link any dependencies
add_subdirectory(
)

target_link_libraries(rtk_baja
    # Add any required libraries here
)