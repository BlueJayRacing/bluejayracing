# Minimum required CMake version
cmake_minimum_required(VERSION 3.28)

# get all of the source and headers by globbing through the src directory
file(GLOB_RECURSE SOURCES "src/*.cpp")
file(GLOB_RECURSE HEADERS "include/**/*.h")
add_library(baja_proto STATIC ${SOURCES} ${HEADERS})
target_include_directories(baja_proto PUBLIC 
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
    TARGET baja_proto 
    PROTOS ${CMAKE_CURRENT_SOURCE_DIR}/proto/baja_live_comm.proto
    PROTOC_OUT_DIR ${CMAKE_CURRENT_BINARY_DIR}
    PROTOC_OPTIONS --experimental_allow_proto3_optional
    LANGUAGE cpp
)

# Recursively link any dependencies
target_link_libraries(baja_proto
    PUBLIC ${PROTOBUF_LIBRARIES}
    # Add any required libraries here
)