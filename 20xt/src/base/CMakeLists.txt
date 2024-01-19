cmake_minimum_required(VERSION 3.12)
project(StationMain VERSION 1.0.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_COLOR_DIAGNOSTICS ON)
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# TESTING
add_executable(testing src/testing.cpp)

# LOGGER
add_executable(logger src/logger.cpp)

# REMOTE COMM 
add_executable(remote_comm src/remote_comm.cpp)
add_subdirectory(
    ${CMAKE_CURRENT_SOURCE_DIR}/../xbee_baja
    ${CMAKE_CURRENT_SOURCE_DIR}/../xbee_baja/bin
)
target_link_libraries(remote_comm PUBLIC xbee_baja) 