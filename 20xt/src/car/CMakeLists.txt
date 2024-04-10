cmake_minimum_required(VERSION 3.12)
project(CarMains VERSION 1.0.0)

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
    ${CMAKE_CURRENT_SOURCE_DIR}/../paho.mqtt.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../paho.mqtt.c/bin
)

add_subdirectory(
    ${CMAKE_CURRENT_SOURCE_DIR}/../ads1115
    ${CMAKE_CURRENT_SOURCE_DIR}/../ads1115/bin
)

# TEST - ADC DRIVER
add_executable(adc_driver_test tests/adc_driver_test.cpp ${HELPER_SOURCES})
target_link_libraries(adc_driver_test 
    PUBLIC baja_proto  
    PUBLIC baja_ipc    
)


# ADC DRIVER
add_executable(adc_driver src/adc_driver.cpp ${HELPER_SOURCES})
target_link_libraries(adc_driver 
    PUBLIC baja_proto  
    PUBLIC baja_ipc    
    PUBLIC ads1115
)

# MQTT CLIENT
add_executable(mqtt_client src/mqtt_client.cpp ${HELPER_SOURCES})
target_link_libraries(mqtt_client
    PUBLIC paho-mqtt3c
    PUBLIC baja_proto  
    PUBLIC baja_ipc 
)

# BROKER
add_executable(broker src/broker.cpp ${HELPER_SOURCES})
target_link_libraries(broker
    PUBLIC baja_proto  
    PUBLIC baja_ipc 
)

# SD WRITER
add_executable(sd_writer src/sd_writer.cpp ${HELPER_SOURCES})
target_link_libraries(sd_writer
    PUBLIC baja_proto  
    PUBLIC baja_ipc 
)

# TRANSMIT PRIORITIZER
add_executable(transmit_prioritizer src/transmit_prioritizer.cpp ${HELPER_SOURCES})
target_link_libraries(transmit_prioritizer
    PUBLIC baja_proto  
    PUBLIC baja_ipc
    PUBLIC xbee_baja
)

# XBEE DRIVER
add_executable(xbee_driver src/xbee_driver.cpp ${HELPER_SOURCES})
target_link_libraries(xbee_driver
    PUBLIC baja_proto  
    PUBLIC baja_ipc 
    PUBLIC xbee_baja
)
