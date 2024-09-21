cmake_minimum_required(VERSION 3.28)
project(inertial_sense_ros_humble)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(inertial_sense_ros_humble_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Threads REQUIRED)
# find_package(inertial_sense_sdk REQUIRED)

include_directories(include
  ${YAML_CPP_INCLUDE_DIR}
  lib/inertial-sense-sdk/src
  lib/inertial-sense-sdk/src/libusb
  include
)

add_subdirectory(lib/inertial-sense-sdk lib/inertial-sense-sdk/src/libusb)

add_library(inertial_sense_ros
  src/inertial_sense_ros_complete.cpp
  
)
target_link_libraries(inertial_sense_ros InertialSense ${YAML_CPP_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})
# target_link_libraries(inertial_sense_ros ${YAML_CPP_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})
ament_target_dependencies(inertial_sense_ros
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  std_srvs
  diagnostic_msgs
  tf2
  tf2_ros
  # inertial_sense_sdk
  inertial_sense_ros_humble_msgs
)



add_executable(inertial_sense_node src/inertial_sense_node.cpp)
target_link_libraries(inertial_sense_node inertial_sense_ros)
ament_target_dependencies(inertial_sense_node
  rclcpp
)



install(TARGETS inertial_sense_ros inertial_sense_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()