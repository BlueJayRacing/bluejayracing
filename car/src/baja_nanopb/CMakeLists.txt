cmake_minimum_required(VERSION 3.28)
project(baja_nanopb)

# Find required packages.
find_package(ament_cmake REQUIRED)
find_package(Python3 COMPONENTS Interpreter REQUIRED)
find_package(nanopb REQUIRED)

# Locate the nanopb generator script. It must be available in your PATH.
find_program(NANOPB_GENERATOR nanopb_generator.py)
if(NOT NANOPB_GENERATOR)
  message(FATAL_ERROR "nanopb_generator.py not found. Ensure nanopb is installed via apt or available in your PATH.")
endif()

# Gather all .proto files in the proto directory.
file(GLOB PROTO_FILES "${CMAKE_CURRENT_SOURCE_DIR}/proto/*.proto")

message(PROTO_FILES="$PROTO_FILES")

# Lists to hold generated source and header files.
set(GENERATED_SRCS "")
set(GENERATED_HDRS "")

foreach(PROTO_FILE ${PROTO_FILES})
  # Get the base name of the proto file.
  get_filename_component(PROTO_NAME ${PROTO_FILE} NAME_WE)
  set(GENERATED_C ${CMAKE_CURRENT_BINARY_DIR}/${PROTO_NAME}.pb.c)
  set(GENERATED_H ${CMAKE_CURRENT_BINARY_DIR}/${PROTO_NAME}.pb.h)

  # Generate the nanopb C files.
  add_custom_command(
    OUTPUT ${GENERATED_C} ${GENERATED_H}
    COMMAND ${Python3_EXECUTABLE} ${NANOPB_GENERATOR} -I${CMAKE_CURRENT_SOURCE_DIR}/proto ${PROTO_FILE} --output-dir=${CMAKE_CURRENT_BINARY_DIR}
    DEPENDS ${PROTO_FILE}
    COMMENT "Generating nanopb C files from ${PROTO_FILE}"
  )
  list(APPEND GENERATED_SRCS ${GENERATED_C})
  list(APPEND GENERATED_HDRS ${GENERATED_H})
endforeach()

# Create a custom target to ensure code generation runs.
add_custom_target(generate_nanopb_code ALL DEPENDS ${GENERATED_SRCS} ${GENERATED_HDRS})

# Build the shared library from the generated sources.
add_library(${PROJECT_NAME} SHARED
  ${GENERATED_SRCS}
)
# Ensure the library waits for code generation.
add_dependencies(${PROJECT_NAME} generate_nanopb_code)

# Add include directories:
# - The build directory (for the generated headers).
# - The nanopb headers directory's "nanopb" subfolder (so that "pb.h" is found).
target_include_directories(${PROJECT_NAME} PUBLIC
  ${CMAKE_CURRENT_BINARY_DIR}
  ${nanopb_INCLUDE_DIRS}/nanopb
)

# Link against the nanopb runtime.
target_link_libraries(${PROJECT_NAME} PUBLIC)

ament_target_dependencies(${PROJECT_NAME} nanopb)

# Install the generated header files.
install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.pb.h"
)

# Install the shared library.
install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(${CMAKE_CURRENT_BINARY_DIR})
ament_export_libraries(${PROJECT_NAME})

ament_package()
