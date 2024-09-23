cmake_minimum_required(VERSION 3.28)
project(comms)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# Include directories
include_directories(
  include
)

# Create the libraries
add_library(i2c_dev SHARED
  src/i2c_dev.cpp
)
add_library(spi_dev SHARED
  src/spi.cpp
)

# Set target properties
ament_target_dependencies(i2c_dev
  rclcpp
)

# Install targets
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(TARGETS i2c_dev spi_dev LIBRARY DESTINATION lib)


ament_export_include_directories(include)

ament_package()