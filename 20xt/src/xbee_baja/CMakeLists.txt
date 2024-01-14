# Minimum required CMake version
cmake_minimum_required(VERSION 3.28)

# get all of the source and headers by globbing through the src directory
file(GLOB_RECURSE SOURCES "src/*.cpp")
file(GLOB_RECURSE HEADERS "include/**/*.h")

# Create library target
add_library(xbee_baja STATIC ${SOURCES} ${HEADERS} ${PROTO_SRCS} ${PROTO_HDRS})
target_include_directories(xbee_baja PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_BINARY_DIR}/proto
    ${CMAKE_CURRENT_BINARY_DIR}
)

# Remove generated protobuf headers if they were manually generated
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/proto/baja_live_comm.pb.h)
    file(REMOVE ${CMAKE_CURRENT_SOURCE_DIR}/proto/baja_live_comm.pb.h)
endif()
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/proto/baja_live_comm.pb.cc)
    file(REMOVE ${CMAKE_CURRENT_SOURCE_DIR}/proto/baja_live_comm.pb.cc)
endif()

# Generate protobuf headers
find_package(Protobuf REQUIRED)
protobuf_generate(
    TARGET xbee_baja 
    PROTOS ${CMAKE_CURRENT_SOURCE_DIR}/proto/baja_live_comm.proto
    PROTOC_OUT_DIR ${CMAKE_CURRENT_BINARY_DIR}
    PROTOC_OPTIONS --experimental_allow_proto3_optional
    LANGUAGE cpp
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