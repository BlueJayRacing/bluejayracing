# Minimum required CMake version
cmake_minimum_required(VERSION 3.12)

set(SOURCES
    src/queue_manage.cpp 
    src/safe_queue.cpp 
    src/station_driver.cpp 
    src/xbee_connection.cpp
    # Add more source files here
)

set(HEADERS
    include/connection.h
    include/guard.h
    include/queue_manage.h
    include/safe_queue.h
    include/station_driver.h
    include/trx_queues.h
    include/xbee_connection.h
    # Add library header files
)

# Generate protobuf files
include(FindProtobuf)
find_package(Protobuf REQUIRED)
include_directories(${PROTOBUF_INCLUDE_DIR})
include_directories(${CMAKE_CURRENT_BINARY_DIR})
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS proto/baja_live_comm.proto)

# Create library target
add_library(xbee_baja STATIC ${SOURCES} ${HEADERS} ${PROTO_SRCS} ${PROTO_HDRS})

target_include_directories(xbee_baja PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/proto
    # TODO: It will be necessary to have the proto files build with the PROTOBUF_GENERATE_CPP function
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