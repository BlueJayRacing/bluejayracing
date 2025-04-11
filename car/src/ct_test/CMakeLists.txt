cmake_minimum_required(VERSION 3.8)
project(ct_test)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# Find CT dependencies
find_package(ct_core REQUIRED)
find_package(ct_optcon REQUIRED)
find_package(ct_rbd REQUIRED)

# Include directories
include_directories(
  include
  ${ct_core_INCLUDE_DIRS}
  ${ct_optcon_INCLUDE_DIRS}
  ${ct_rbd_INCLUDE_DIRS}
)

# CT Core example executable
add_executable(test_ct_core src/test_ct_core.cpp)
ament_target_dependencies(test_ct_core rclcpp ct_core)

# CT Optcon example executable
add_executable(test_ct_optcon src/test_ct_optcon.cpp)
ament_target_dependencies(test_ct_optcon rclcpp ct_core ct_optcon)

# Test prioritize
add_executable(test_prioritize src/test_prioritize.cpp)
ament_target_dependencies(test_prioritize rclcpp ct_core ct_optcon)


# Install executables
install(TARGETS
  test_ct_core
  test_ct_optcon
  test_prioritize
  DESTINATION lib/${PROJECT_NAME}
)

# Install include directory
install(DIRECTORY include/
  DESTINATION include/
)

ament_package()