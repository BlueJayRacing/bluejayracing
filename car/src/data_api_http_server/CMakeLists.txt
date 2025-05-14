cmake_minimum_required(VERSION 3.8)
project(data_api_http_server)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)


find_package(pistache QUIET)
if(NOT pistache_FOUND)
  message(STATUS "Could not find Pistache via find_package, falling back to system libs.")
endif()
find_package(nlohmann_json REQUIRED)
find_package(baja_msgs REQUIRED)

include_directories(
  include
)

# Build the library for the aggregation server
add_library(aggregation_server_lib
  src/aggregator.cpp
  src/http_handler.cpp
  src/aggregation_server.cpp
)
ament_target_dependencies(aggregation_server_lib rclcpp std_msgs nlohmann_json baja_msgs)
if(pistache_FOUND)
  target_link_libraries(aggregation_server_lib ${pistache_LIBRARIES})
else()
  target_link_libraries(aggregation_server_lib pistache)
endif()

# Build the executable for the aggregation server node
add_executable(aggregation_server_node
  src/aggregation_server_node.cpp
)
ament_target_dependencies(aggregation_server_node rclcpp baja_msgs)
target_link_libraries(aggregation_server_node aggregation_server_lib)

install(TARGETS
  aggregation_server_lib
  aggregation_server_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
