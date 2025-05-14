cmake_minimum_required(VERSION 3.8)
project(baja_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Automatically find all message files in the msg directory
file(GLOB MESSAGE_FILES "${CMAKE_CURRENT_SOURCE_DIR}/msg/*.msg")

# Convert absolute paths to the required format: <absolute_base_path>:<relative_path>
set(MSGS)
foreach(msg_file ${MESSAGE_FILES})
  file(RELATIVE_PATH rel_msg_file ${CMAKE_CURRENT_SOURCE_DIR} ${msg_file})
  list(APPEND MSGS "${CMAKE_CURRENT_SOURCE_DIR}:${rel_msg_file}")
endforeach()

set(ACTIONS)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSGS}
  ${ACTIONS}
  DEPENDENCIES geometry_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Skipping linter checks for copyrights
  set(ament_cmake_copyright_FOUND TRUE)
  # Skipping cpplint (only works in a git repo)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)

ament_package()

