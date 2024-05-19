# Minimum required CMake version
cmake_minimum_required(VERSION 3.28)
project(xbee_baja VERSION 1.0.0)

# get all of the source and headers by globbing through the src directory
if (NOT TARGET xbee_baja)
    file(GLOB_RECURSE SOURCES "src/*")
    file(GLOB_RECURSE HEADERS "include/**/*.h")
    add_library(xbee_baja STATIC ${SOURCES} ${HEADERS})
    target_include_directories(xbee_baja PUBLIC 
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_BINARY_DIR}/proto
        ${CMAKE_CURRENT_BINARY_DIR}
    )

    # Recursively link any dependencies
    add_subdirectory(
        ${CMAKE_CURRENT_SOURCE_DIR}/../xbee_digi
        ${CMAKE_CURRENT_SOURCE_DIR}/../xbee_digi/bin
    )
    target_link_libraries(xbee_baja
        PUBLIC xbee_digi
        # Add any required libraries here
    )
endif()