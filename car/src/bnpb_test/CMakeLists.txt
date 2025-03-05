cmake_minimum_required(VERSION 3.28)
project(bnpb_test)

find_package(ament_cmake REQUIRED)
find_package(baja_nanopb REQUIRED)
find_package(nanopb REQUIRED)

add_executable(consumer
  src/consumer.c
)

# Link against the library from my_wsg_cal_package and also the nanopb runtime.
ament_target_dependencies(consumer
  baja_nanopb
  nanopb
)

# The old package exports a library and includes. Linking ensures we get the generated headers.
target_link_libraries(consumer

)

install(TARGETS consumer
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
