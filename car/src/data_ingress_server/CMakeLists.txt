cmake_minimum_required(VERSION 3.5)
project(data_ingress_server)

# Default to C++14 if not already set
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
)

# Build the library
add_library(udp_hello_world_lib
  src/udp_hello_world.cpp
)
target_link_libraries(udp_hello_world_lib
  ${Boost_LIBRARIES}
)
ament_target_dependencies(udp_hello_world_lib rclcpp)

# Build the node executable (that simply instantiates the library)
add_executable(udp_hello_world_node
  src/udp_hello_world_node.cpp
)
target_link_libraries(udp_hello_world_node
  udp_hello_world_lib
  ${Boost_LIBRARIES}
)
ament_target_dependencies(udp_hello_world_node rclcpp)

install(TARGETS
  udp_hello_world_lib
  udp_hello_world_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_package()
