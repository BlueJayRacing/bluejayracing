# Minimum required CMake version
cmake_minimum_required(VERSION 3.28)
project(data_logging VERSION 1.0.0)

cmake_minimum_required(VERSION 3.12)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Library subdirectories
add_subdirectory(
    ${CMAKE_CURRENT_SOURCE_DIR}/../
    ${CMAKE_CURRENT_SOURCE_DIR}/../test_bin
)

# TESTING CODES
add_executable(test_write_configs test_write_configs.cpp ${HELPER_SOURCES})
target_link_libraries(test_write_configs
    PUBLIC data_logging
)
