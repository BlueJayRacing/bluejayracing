cmake_minimum_required(VERSION 3.28)
project(uart_emit)

set(CMAKE_COLOR_DIAGNOSTICS ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(baja_proto REQUIRED)
find_package(baja_msgs REQUIRED)

# Include directories
include_directories(
  include
)

# Create uart_emit library
add_library(uart_emit SHARED
  src/uart_emit.cpp
)
ament_target_dependencies(uart_emit
  rclcpp
  baja_msgs
)

# Create node executable
add_executable(uart_emit_node
  src/node.cpp
)
target_link_libraries(uart_emit_node
  uart_emit
)


install( DIRECTORY include/ DESTINATION include )
install( TARGETS uart_emit LIBRARY DESTINATION lib )
install( TARGETS uart_emit_node DESTINATION lib/${PROJECT_NAME} )


ament_export_include_directories(include)

# Call ament_package() to generate necessary files
ament_package()