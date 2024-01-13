# Minimum required CMake version
cmake_minimum_required(VERSION 3.12)

set(SOURCES
    src/queue_manage.cpp 
    src/station_driver.cpp 
    src/xbee_connection.cpp
    src/trx_queues.cpp
    # Add more source files here
)

set(HEADERS
    include/interfaces/connection.h
    include/ipc/queue_manage.h
    include/ipc/safe_queue.h
    include/ipc/trx_queues.h
    include/mains/station_driver.h
    include/mains/trx_dispatcher.h
    include/xbee/car_serial_config.h
    include/xbee/station_serial_config.h
    include/xbee/xbee_baja_network_config.h
    include/xbee/xbee_connection.h
    # Add library header files
)

# Remove generated protobuf headers if they exist
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/proto/baja_live_comm.pb.h)
    file(REMOVE ${CMAKE_CURRENT_SOURCE_DIR}/proto/baja_live_comm.pb.h)
endif()
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/proto/baja_live_comm.pb.cc)
    file(REMOVE ${CMAKE_CURRENT_SOURCE_DIR}/proto/baja_live_comm.pb.cc)
endif()

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
    ${CMAKE_CURRENT_BINARY_DIR}
    # TODO: It will be necessary to have the proto files build with the PROTOBUF_GENERATE_CPP function
)

# Recursively link any dependencies
add_subdirectory(
    ${CMAKE_CURRENT_SOURCE_DIR}/../xbee_digi
    ${CMAKE_CURRENT_SOURCE_DIR}/../xbee_digi/bin
)

target_link_libraries(xbee_baja
    PUBLIC xbee_digi
    PUBLIC ${PROTOBUF_LIBRARIES}
    # Add any required libraries here
)