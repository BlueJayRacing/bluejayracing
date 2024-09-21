cmake_minimum_required(VERSION 3.28)
project(xbee_baja)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(xbee_digi REQUIRED)

include_directories(
  include
  # ${xbee_digi_INCLUDE_DIRS}
)


file(GLOB_RECURSE CPP_SOURCES "src/*.cpp")
file(GLOB_RECURSE C_SOURCES "src/*.c")
file(GLOB_RECURSE HEADERS "include/**/*.h")

# target_compile_definitions(xbee_baja PUBLIC "POSIX")

add_library( xbee_baja SHARED ${C_SOURCES} ${CPP_SOURCES} )



target_compile_definitions(xbee_baja PUBLIC "POSIX")


target_include_directories(xbee_baja PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(xbee_baja
  xbee_digi
)

# Uncomment and add any required libraries here
# target_link_libraries(xbee_baja
#   some_library
# )

install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS xbee_baja
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

# Uncomment if you have any tests
# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   ament_lint_auto_find_test_dependencies()
# endif()

ament_export_include_directories(include)
ament_export_libraries(xbee_baja)
ament_export_dependencies(xbee_digi)

ament_package()