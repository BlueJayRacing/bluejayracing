cmake_minimum_required(VERSION 3.28)
project(car_config)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

install(FILES config/car_config.json
        DESTINATION share/${PROJECT_NAME}/config)

ament_package()