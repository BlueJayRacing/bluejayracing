cmake_minimum_required(VERSION 3.28)
project(baja_proto)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Protobuf REQUIRED)

# Create Temp Folder
file(MAKE_DIRECTORY "bin")

# Add library
# add_library(baja_proto_gen OBJECT ${CMAKE_CURRENT_SOURCE_DIR}/proto)
add_library(baja_proto_gen OBJECT )
target_include_directories(baja_proto_gen PUBLIC 
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/bin>"
)
target_link_libraries(baja_proto_gen PUBLIC protobuf::libprotobuf)

# Include directories
include_directories(
  include
  ${CMAKE_CURRENT_BINARY_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}/proto
)

# Generate protobuf messages
protobuf_generate(
  TARGET baja_proto_gen
  PROTOS "proto/baja_live_comm.proto"
  PROTOC_OUT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/bin
  PROTOC_OPTIONS --experimental_allow_proto3_optional
  LANGUAGE cpp
)

# Add custom command to generate protobuf files
add_custom_command(
  OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/bin/proto/baja_live_comm.pb.cc
         ${CMAKE_CURRENT_SOURCE_DIR}/bin/proto/baja_live_comm.pb.h
  COMMAND protobuf::protoc
  ARGS --cpp_out=${CMAKE_CURRENT_SOURCE_DIR}/bin
       --proto_path=${CMAKE_CURRENT_SOURCE_DIR}/proto/baja_live_comm.proto
  DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/proto/baja_live_comm.proto
)

# Add custom target to trigger protobuf generation
add_custom_target(
  baja_proto_generate
  DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/bin/proto/baja_live_comm.pb.cc
          ${CMAKE_CURRENT_SOURCE_DIR}/bin/proto/baja_live_comm.pb.h
)

# Add library
include_directories( bin bin/proto )
add_library(baja_proto SHARED
  ${CMAKE_CURRENT_SOURCE_DIR}/bin/proto/baja_live_comm.pb.cc
)
target_include_directories(baja_proto
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/bin>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(baja_proto
  PUBLIC protobuf::libprotobuf
)
add_dependencies(baja_proto baja_proto_generate)

# Install targets
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/proto/ DESTINATION share/${PROJECT_NAME})
install(TARGETS baja_proto LIBRARY DESTINATION lib)

ament_export_include_directories(${CMAKE_CURRENT_SOURCE_DIR}/bin)
ament_export_libraries(baja_proto)

ament_package()