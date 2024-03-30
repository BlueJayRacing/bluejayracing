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
add_subdirectory(
    ${CMAKE_CURRENT_SOURCE_DIR}/../baja_ipc
    ${CMAKE_CURRENT_SOURCE_DIR}/../baja_ipc/bin
)
add_subdirectory(
    ${CMAKE_CURRENT_SOURCE_DIR}/../rtk
    ${CMAKE_CURRENT_SOURCE_DIR}/../rtk/bin
)

# Helping code
file(GLOB_RECURSE HELPER_SOURCES "src/helpers/*.cpp")

# TESTING CODES
add_executable(test_produce_pit_commands tests/test_produce_pit_commands.cpp ${HELPER_SOURCES})
target_link_libraries(test_produce_pit_commands
    PUBLIC xbee_baja
    PUBLIC baja_proto  
    PUBLIC baja_ipc
    PUBLIC rtk
)

add_executable(test_stress_rx_dispatcher tests/test_stress_rx_dispatcher.cpp ${HELPER_SOURCES})
target_link_libraries(test_stress_rx_dispatcher
    PUBLIC xbee_baja
    PUBLIC baja_proto    
    PUBLIC baja_ipc  
    PUBLIC rtk
)

add_executable(test_stress_tx_prioritizer tests/test_stress_tx_prioritizer.cpp ${HELPER_SOURCES})
target_link_libraries(test_stress_tx_prioritizer
    PUBLIC xbee_baja
    PUBLIC baja_proto 
    PUBLIC baja_ipc  
    PUBLIC rtk   
)

# PIT COMMANDS
add_executable(pit_commands src/pit_commands.cpp ${HELPER_SOURCES})
target_link_libraries(pit_commands 
    PUBLIC xbee_baja
    PUBLIC baja_proto  
    PUBLIC baja_ipc  
    PUBLIC rtk  
)

# RTK Corrections
add_executable(rtk_corrections src/rtk_corrections.cpp ${HELPER_SOURCES})
target_link_libraries(rtk_corrections 
    PUBLIC xbee_baja
    PUBLIC baja_proto 
    PUBLIC baja_ipc  
    PUBLIC rtk   
)

# TRANSMIT PRIORITIZER
add_executable(transmit_prioritizer src/transmit_prioritizer.cpp ${HELPER_SOURCES})
target_link_libraries(transmit_prioritizer
    PUBLIC xbee_baja
    PUBLIC baja_proto   
    PUBLIC baja_ipc
    PUBLIC rtk   
)

# XBEE DRIVER 
add_executable(xbee_driver src/xbee_driver.cpp ${HELPER_SOURCES})
target_link_libraries(xbee_driver 
    PUBLIC xbee_baja
    PUBLIC baja_proto  
    PUBLIC baja_ipc    
    PUBLIC rtk
) 

# RECIEVE DISPATCHER
add_executable(receive_dispatcher src/receive_dispatcher.cpp ${HELPER_SOURCES})
target_link_libraries(receive_dispatcher 
    PUBLIC xbee_baja
    PUBLIC baja_proto 
    PUBLIC baja_ipc   
    PUBLIC rtk  
)

# LOGGER
add_executable(data_logger src/data_logger.cpp ${HELPER_SOURCES})
target_link_libraries(data_logger 
    PUBLIC xbee_baja
    PUBLIC baja_proto
    PUBLIC baja_ipc   
    PUBLIC rtk   
)