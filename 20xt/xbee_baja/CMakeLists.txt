# Minimum required CMake version
cmake_minimum_required(VERSION 3.12)

set(SOURCES
    # Add more source files here
)

set(HEADERS
    # Add library header files
)

# Create library target
add_library(xbee_baja STATIC ${SOURCES} ${HEADERS})

target_include_directories(xbee_baja PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Recursively link any dependencies
add_subdirectory(
    ${CMAKE_CURRENT_SOURCE_DIR}/../xbee_digi
    ${CMAKE_CURRENT_SOURCE_DIR}/../xbee_digi/bin
)

target_link_libraries(xbee_baja
    PRIVATE xbee_digi
    # Add any required libraries here
)