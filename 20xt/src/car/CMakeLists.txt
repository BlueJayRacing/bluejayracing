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

# Helping code
file(GLOB_RECURSE HELPER_SOURCES "src/helpers/*.cpp")

# ADC DRIVER
add_executable(adc_driver src/adc_driver.cpp ${HELPER_SOURCES})
target_link_libraries(pit_commands 
    PUBLIC baja_proto  
    PUBLIC baja_ipc    
)

