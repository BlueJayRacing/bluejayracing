cmake_minimum_required(VERSION 3.28)
project(car)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()



# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(xbee_baja REQUIRED)
find_package(baja_proto REQUIRED)
find_package(baja_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(paho_mqtt REQUIRED)
find_package(ads1115 REQUIRED)
find_package(inertial_sense_ros_humble REQUIRED)
find_package(inertial_sense_ros_humble_msgs REQUIRED)

# Include directories
include_directories(
  include
)


# TEST - ADC DRIVER
# add_executable(adc_driver_test src/adc_driver_test.cpp ${HELPER_SOURCES})
# ament_target_dependencies(adc_driver_test
#   rclcpp
#   baja_proto
#   ads1115
# )


# # MQTT CLIENT SUBSCRIBE
add_library(mqtt_client_driver SHARED src/mqtt_client_driver.cpp)
ament_target_dependencies(mqtt_client_driver
  rclcpp
  baja_msgs
  baja_proto
  paho_mqtt
)
add_executable(mqtt_client_node src/mqtt_client_node.cpp)
target_link_libraries(mqtt_client_node mqtt_client_driver)
ament_target_dependencies(mqtt_client_node
  rclcpp
  baja_msgs
  baja_proto
  paho_mqtt
)

# MQTT CLIENT PUBLISH
add_library(mqtt_client_publish_driver SHARED src/mqtt_client_publish_driver.cpp)
ament_target_dependencies(mqtt_client_publish_driver
  rclcpp
  baja_msgs
  baja_proto
  paho_mqtt
)
add_executable(mqtt_client_publish_node src/mqtt_client_publish_node.cpp ${HELPER_SOURCES})
target_link_libraries(mqtt_client_publish_node mqtt_client_publish_driver)
ament_target_dependencies(mqtt_client_publish_node
  paho_mqtt
  baja_msgs
  baja_proto
  rclcpp
)

# BROKER
add_library(broker_driver SHARED src/broker_driver.cpp)
ament_target_dependencies(broker_driver
  rclcpp
  baja_msgs
  baja_proto
)
add_executable(broker_node src/broker_node.cpp)
target_link_libraries(broker_node broker_driver)
ament_target_dependencies(broker_node
  rclcpp
  baja_msgs
  baja_proto
)

# DISC WRITER
add_library(writer_driver SHARED src/writer_driver.cpp)
ament_target_dependencies(writer_driver
  rclcpp
  baja_msgs
  baja_proto
  inertial_sense_ros_humble
  inertial_sense_ros_humble_msgs
)
add_executable(writer_node src/writer_node.cpp)
target_link_libraries(writer_node writer_driver)
ament_target_dependencies(writer_node
  rclcpp
  baja_msgs
  baja_proto
)

# TRANSMIT PRIORITIZER
add_library(transmit_prioritizer_driver SHARED src/transmit_prioritizer_driver.cpp)
target_compile_definitions(transmit_prioritizer_driver PUBLIC "POSIX")
ament_target_dependencies(transmit_prioritizer_driver
  rclcpp
  baja_msgs
  baja_proto
  xbee_baja
  std_msgs
)
add_executable(transmit_prioritizer_node src/transmit_prioritizer_node.cpp)
target_link_libraries(transmit_prioritizer_node transmit_prioritizer_driver)
ament_target_dependencies(transmit_prioritizer_node
  rclcpp
  baja_msgs
  baja_proto
  xbee_baja
  std_msgs
)

# XBEE DRIVER
add_library(xbee_driver SHARED src/xbee_driver.cpp)
target_compile_definitions(xbee_driver PUBLIC "POSIX")
ament_target_dependencies(xbee_driver
  rclcpp
  baja_msgs
  baja_proto
  xbee_baja
  std_msgs
)
add_executable(xbee_node src/xbee_node.cpp)
target_link_libraries(xbee_node xbee_driver)
ament_target_dependencies(xbee_node
  rclcpp
  baja_msgs
  baja_proto
  xbee_baja
  std_msgs
)

# Calibration Printer
add_library(calibration_printer SHARED src/calibration_printer.cpp)
ament_target_dependencies(calibration_printer
  rclcpp
  baja_msgs
)
add_executable(calibration_printer_node src/calibration_printer_node.cpp)
target_link_libraries(calibration_printer_node calibration_printer)
ament_target_dependencies(calibration_printer_node
  rclcpp
  baja_msgs
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
  broker_driver
  writer_driver
  mqtt_client_driver
  mqtt_client_publish_driver
  transmit_prioritizer_driver
  xbee_driver
  calibration_printer
  LIBRARY DESTINATION lib
)

# Install targets
install(TARGETS
  mqtt_client_node
  mqtt_client_publish_node
  broker_node
  writer_node
  transmit_prioritizer_node
  xbee_node
  calibration_printer_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)

ament_package()