cmake_minimum_required(VERSION 3.28)
project(udp_data_chunk_server)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(baja_nanopb REQUIRED)
find_package(nanopb REQUIRED)
find_package(baja_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

add_executable(udp_data_chunk_server
  src/udp_data_chunk_server.cpp
)

# Link against the library from my_wsg_cal_package and also the nanopb runtime.
ament_target_dependencies(udp_data_chunk_server
  rclcpp
  baja_nanopb
  nanopb
  baja_msgs
  ament_index_cpp
)

# The old package exports a library and includes. Linking ensures we get the generated headers.
target_link_libraries(udp_data_chunk_server

)

install(TARGETS udp_data_chunk_server
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
