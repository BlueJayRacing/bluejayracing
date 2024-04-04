# Minimum required CMake version
cmake_minimum_required(VERSION 3.28)
project(ads1115 VERSION 1.0.0)

# get all of the source and headers by globbing through the src directory

if(NOT TARGET ads1115)
    file(GLOB_RECURSE SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")
    file(GLOB_RECURSE HEADERS "${CMAKE_CURRENT_SOURCE_DIR}/include/**/*.h")
    add_library(ads1115 STATIC ${SOURCES} ${HEADERS})
    target_include_directories(ads1115 PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_BINARY_DIR}
    )

    add_subdirectory(
        ${CMAKE_CURRENT_SOURCE_DIR}/../comms
        ${CMAKE_CURRENT_SOURCE_DIR}/../comms/bin
    )

    target_link_libraries(ads1115
    PUBLIC comms
    # Add any required libraries here
    )
endif()