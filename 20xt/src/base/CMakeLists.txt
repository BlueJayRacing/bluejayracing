cmake_minimum_required(VERSION 3.12)
project(StationMain VERSION 1.0.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_COLOR_DIAGNOSTICS ON)
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
# Library subdirectories
add_subdirectory(
    ${CMAKE_CURRENT_SOURCE_DIR}/../xbee_baja
    ${CMAKE_CURRENT_SOURCE_DIR}/../xbee_baja/bin
)

# PIT COMMANDS
add_executable(pit_commands src/pit_commands.cpp)
target_link_libraries(pit_commands PUBLIC xbee_baja)

# LOGGER
add_executable(logger src/logger.cpp)
target_link_libraries(logger PUBLIC xbee_baja)

# REMOTE COMM 
add_executable(remote_comm src/remote_comm.cpp)
target_link_libraries(remote_comm PUBLIC xbee_baja) 