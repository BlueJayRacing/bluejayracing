cmake_minimum_required(VERSION 3.8)
project(data_api_server)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Pistache might have a CMake config file named 'pistache'.
# If it doesn't, you can still include its headers and link 
# its library if needed. For example:
find_package(pistache QUIET)
if(NOT pistache_FOUND)
  message(STATUS "Could not find Pistache via find_package, falling back to system libs.")
  # Typically, you'd just link to -lpistache if needed:
  # set(Pistache_LIBRARIES "pistache")
endif()

include_directories(
  include
)

# Build the library for the server node
add_library(pistache_hello_world
  src/pistache_hello_world.cpp
)
ament_target_dependencies(pistache_hello_world rclcpp)
if(pistache_FOUND)
  target_link_libraries(pistache_hello_world ${pistache_LIBRARIES})
else()
  target_link_libraries(pistache_hello_world pistache)
endif()

# Build the executable for the main node
add_executable(pistache_hello_world_node
  src/pistache_hello_world_node.cpp
)
ament_target_dependencies(pistache_hello_world_node rclcpp)
target_link_libraries(pistache_hello_world_node pistache_hello_world)




# Build the library for the server node
add_library(pistache_realtime_data_api_server
  src/pistache_realtime_data_api_server.cpp
)
ament_target_dependencies(pistache_realtime_data_api_server rclcpp std_msgs)
if(pistache_FOUND)
  target_link_libraries(pistache_realtime_data_api_server ${pistache_LIBRARIES})
else()
  target_link_libraries(pistache_realtime_data_api_server pistache)
endif()

# Build the executable for the main node
add_executable(pistache_realtime_data_api_server_node
  src/pistache_realtime_data_api_server_main.cpp
)
ament_target_dependencies(pistache_realtime_data_api_server_node rclcpp std_msgs)
target_link_libraries(pistache_realtime_data_api_server_node pistache_realtime_data_api_server)



# Build the library for the mock data API server
add_library(mock_data_api_server
  src/mock_data_api_server.cpp
)
ament_target_dependencies(mock_data_api_server rclcpp std_msgs)
if(pistache_FOUND)
  target_link_libraries(mock_data_api_server ${pistache_LIBRARIES})
else()
  target_link_libraries(mock_data_api_server pistache)
endif()
# Add nlohmann/json if found
if(nlohmann_json_FOUND)
  target_link_libraries(mock_data_api_server nlohmann_json::nlohmann_json)
endif()

# Build the executable for the mock data API server node
add_executable(mock_data_api_server_node
  src/mock_data_api_server_main.cpp
)
ament_target_dependencies(mock_data_api_server_node rclcpp std_msgs)
target_link_libraries(mock_data_api_server_node mock_data_api_server)




install(TARGETS
  pistache_hello_world
  pistache_hello_world_node
  pistache_realtime_data_api_server
  pistache_realtime_data_api_server_node
  mock_data_api_server
  mock_data_api_server_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
