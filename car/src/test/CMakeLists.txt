cmake_minimum_required(VERSION 3.28)
project(test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()



# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(baja_proto REQUIRED)
find_package(baja_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(paho_mqtt REQUIRED)
find_package(inertial_sense_ros_humble REQUIRED)
find_package(inertial_sense_ros_humble_msgs REQUIRED)

# Include directories
include_directories(
  include
)

# # MQTT CLIENT SUBSCRIBE
add_library(mqtt_echo_driver SHARED src/mqtt_echo_driver.cpp)
ament_target_dependencies(mqtt_echo_driver
  rclcpp
  baja_msgs
  baja_proto
  paho_mqtt
)
add_executable(mqtt_echo_node src/mqtt_echo_node.cpp)
target_link_libraries(mqtt_echo_node mqtt_echo_driver)
ament_target_dependencies(mqtt_echo_node
  rclcpp
  baja_msgs
  baja_proto
  paho_mqtt
)

# Install headers
install( DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install libraries
install(TARGETS
  mqtt_echo_driver
  LIBRARY DESTINATION lib
)

# Install targets
install(TARGETS
  mqtt_echo_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)

ament_package()