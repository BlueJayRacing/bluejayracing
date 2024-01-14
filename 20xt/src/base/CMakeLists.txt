cmake_minimum_required(VERSION 3.12)
project(StationMain VERSION 1.0.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# LOGGER EXECUTABLE
add_executable(logger src/logger.cpp)


# REMOTE COMM EXECUTABLE 
add_executable(remote_comm src/remote_comm.cpp)

# Turn on color diagnostics
set(CMAKE_COLOR_DIAGNOSTICS ON)

# This top-level CMake needs targets declared in the library's CMake, so
# we use a path to the library's directory
add_subdirectory(
    ${CMAKE_CURRENT_SOURCE_DIR}/../xbee_baja
    ${CMAKE_CURRENT_SOURCE_DIR}/../xbee_baja/bin
)

# By linking the xbee_library CMake target against the my_transmitter target,
# the compiler knows our executable uses code from the library declared in another CMake
target_link_libraries(remote_comm PUBLIC xbee_baja) 

# Q: How can I reference a library that is not under the PROJECT_SOURCE_DIR?