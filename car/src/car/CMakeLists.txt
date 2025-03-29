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
find_package(nanopb REQUIRED)
find_package(baja_nanopb REQUIRED)
find_package(car_config REQUIRED)
find_package(ament_index_cpp REQUIRED)

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

# WSG UPDATE TIME
add_library(wsg_update_time_driver SHARED src/wsg_update_time_driver.cpp)
ament_target_dependencies(wsg_update_time_driver
  rclcpp
  baja_msgs
  paho_mqtt
  nanopb
  baja_nanopb
  baja_proto
)
add_executable(wsg_update_time_node src/wsg_update_time_node.cpp ${HELPER_SOURCES})
target_link_libraries(wsg_update_time_node wsg_update_time_driver)
ament_target_dependencies(wsg_update_time_node
  paho_mqtt
  baja_msgs
  rclcpp
  nanopb
  baja_nanopb
  baja_proto
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

# WSG Drive Data Driver
add_library(wsg_drive_data_driver SHARED src/wsg_drive_data_driver.cpp)
target_compile_definitions(wsg_drive_data_driver PUBLIC "POSIX")
target_include_directories(wsg_drive_data_driver PUBLIC
  ${nanopb_INCLUDE_DIRS}
  "${CMAKE_INSTALL_PREFIX}/include/nanopb"
)
ament_target_dependencies(wsg_drive_data_driver
  car_config
  rclcpp
  baja_msgs
  std_msgs
  paho_mqtt
  nanopb
  baja_nanopb
  baja_proto
  ament_index_cpp
)
add_executable(wsg_drive_data_node src/wsg_drive_data_node.cpp)
target_link_libraries(wsg_drive_data_node wsg_drive_data_driver)
target_include_directories(wsg_drive_data_node PUBLIC
  ${nanopb_INCLUDE_DIRS}
)
ament_target_dependencies(wsg_drive_data_node
  car_config
  rclcpp
  baja_msgs
  std_msgs
  paho_mqtt
  nanopb
  baja_nanopb
  baja_proto
  ament_index_cpp
)

# WSG Calibration Driver
add_library(wsg_calibration_driver SHARED src/wsg_calibration_driver.cpp)
target_compile_definitions(wsg_calibration_driver PUBLIC "POSIX")
target_include_directories(wsg_calibration_driver PUBLIC
  ${nanopb_INCLUDE_DIRS}
  "${CMAKE_INSTALL_PREFIX}/include/nanopb"
)
ament_target_dependencies(wsg_calibration_driver
  rclcpp
  baja_msgs
  std_msgs
  paho_mqtt
  nanopb
  baja_nanopb
)
add_executable(wsg_calibration_node src/wsg_calibration_node.cpp)
target_link_libraries(wsg_calibration_node wsg_calibration_driver)
target_include_directories(wsg_calibration_node PUBLIC
  ${nanopb_INCLUDE_DIRS}
)
ament_target_dependencies(wsg_calibration_node
  rclcpp
  baja_msgs
  std_msgs
  paho_mqtt
  nanopb
  baja_nanopb
)

# # Calibration Printer
# add_library(calibration_printer SHARED src/calibration_printer.cpp)
# ament_target_dependencies(calibration_printer
#   rclcpp
#   baja_msgs
# )
# add_executable(calibration_printer_node src/calibration_printer_node.cpp)
# target_link_libraries(calibration_printer_node calibration_printer)
# ament_target_dependencies(calibration_printer_node
#   rclcpp
#   baja_msgs
# )


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
  transmit_prioritizer_driver
  xbee_driver
  # calibration_printer
  wsg_update_time_driver
  wsg_calibration_driver
  wsg_drive_data_driver
  LIBRARY DESTINATION lib
)

# Install targets
install(TARGETS
  broker_node
  writer_node
  transmit_prioritizer_node
  xbee_node
  # calibration_printer_node
  wsg_update_time_node
  wsg_calibration_node
  wsg_drive_data_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)

ament_package()