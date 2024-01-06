# Minimum required CMake version
cmake_minimum_required(VERSION 3.12)

set(SOURCES
    # Add more source files here
)

set(HEADERS
    # Add library header files
)

# Create library target
add_library(xbee_base_station_funcs STATIC ${SOURCES} ${HEADERS})

# This library has been modified so it is only compatible with POSIX devices
target_include_directories(xbee_base_station_funcs PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# Add the xbee_library as a subdirectory
add_subdirectory(
    ${CMAKE_CURRENT_SOURCE_DIR}/../xbee_ansic_library
    ${CMAKE_CURRENT_SOURCE_DIR}/../xbee_ansic_library/bin
)

target_link_libraries(xbee_base_station_funcs
    # Add any required libraries here
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../xbee_ansic_library
)