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
add_subdirectory(
    ${CMAKE_CURRENT_SOURCE_DIR}/../baja_proto
    ${CMAKE_CURRENT_SOURCE_DIR}/../baja_proto/bin
)

# Helping code
file(GLOB_RECURSE HELPER_SOURCES "src/helpers/*.cpp")

# TESTING
add_executable(test tests/test.cpp ${HELPER_SOURCES})
target_link_libraries(test
    PUBLIC xbee_baja
    PUBLIC baja_proto    
)

# PIT COMMANDS
add_executable(pit_commands src/pit_commands.cpp ${HELPER_SOURCES})
target_link_libraries(pit_commands 
    PUBLIC xbee_baja
    PUBLIC baja_proto    
)

# TRANSMIT PRIORITIZER
add_executable(transmit_prioritizer.cpp src/transmit_prioritizer.cpp ${HELPER_SOURCES})
target_link_libraries(transmit_prioritizer.cpp 
    PUBLIC xbee_baja
    PUBLIC baja_proto    
)

# XBEE DRIVER 
add_executable(xbee_driver src/xbee_driver.cpp ${HELPER_SOURCES})
target_link_libraries(xbee_driver 
    PUBLIC xbee_baja
    PUBLIC baja_proto    
) 

# RECIEVE DISPATCHER
add_executable(receive_dispatcher src/receive_dispatcher.cpp ${HELPER_SOURCES})
target_link_libraries(receive_dispatcher 
    PUBLIC xbee_baja
    PUBLIC baja_proto    
)

# LOGGER
add_executable(data_logger src/data_logger.cpp)
target_link_libraries(data_logger 
    PUBLIC xbee_baja
    PUBLIC baja_proto    
)