cmake_minimum_required(VERSION 3.8)
project(baja_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)


set(MESSAGES
  "msg/AnalogChannel.msg"
  "msg/CarState.msg"
  "msg/Communication.msg"
  "msg/GPS.msg"
  "msg/LiveComm.msg"
  "msg/Localization.msg"
  "msg/Observation.msg"
  "msg/RTKCorrection.msg"
  "msg/Timestamp.msg"
)

set(ACTIONS
)



rosidl_generate_interfaces( ${PROJECT_NAME} ${MESSAGES} ${ACTIONS} DEPENDENCIES geometry_msgs std_msgs )

ament_export_dependencies(rosidl_default_runtime)


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

ament_export_include_directories(include)

ament_package()
