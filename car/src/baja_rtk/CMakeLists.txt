cmake_minimum_required(VERSION 3.28)
project(baja_rtk)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(baja_proto REQUIRED)
find_package(baja_msgs REQUIRED)

# Include directories
include_directories(
  include
)

add_library( serialib SHARED src/serialib.cpp )
ament_target_dependencies( serialib rclcpp )

add_library( rtk_corrections SHARED src/rtk_corrections.cpp )
ament_target_dependencies( rtk_corrections rclcpp baja_proto baja_msgs )

add_executable( rtk_parser src/rtk_parser.cpp )
target_link_libraries( rtk_parser serialib rtk_corrections )
ament_target_dependencies( rtk_parser )

# Install targets
install( TARGETS serialib LIBRARY DESTINATION lib )
install( TARGETS rtk_corrections DESTINATION lib/${PROJECT_NAME} )

ament_export_include_directories(include)

ament_package()