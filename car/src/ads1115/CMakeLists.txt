cmake_minimum_required(VERSION 3.28)
project(ads1115)


# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(baja_msgs REQUIRED)
find_package(baja_proto REQUIRED)
find_package(comms REQUIRED)

# Include directories
include_directories(
  include
)

# Create adafruit_ads library
add_library(adc_driver SHARED
  src/adafruit_ads.cpp
  src/adc.cpp
  src/adc_driver.cpp
  src/adc_node.cpp
  src/i2c_dev.cpp
)
ament_target_dependencies(adc_driver
  comms
  rclcpp
  baja_msgs
  baja_proto
)

# # Create adc library
# add_library(adc SHARED
#   src/adc.cpp
# )
# target_link_libraries(adc
#   adafruit_ads
# )
# ament_target_dependencies(adc
#   comms
# )

# ADC DRIVER

add_executable(adc_node src/adc_node.cpp)
target_link_libraries(adc_node adc_driver)
ament_target_dependencies(adc_node
  rclcpp
  baja_msgs
  baja_proto
)

# Install launch
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)


# Install headers
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# Install libraries
install(TARGETS adc_driver
  LIBRARY DESTINATION lib
)

# Install executables
install(TARGETS adc_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include ${comms_INCLUDE_DIRS})

# Call ament_package()
ament_package()