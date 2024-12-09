# For more information about build system see
# https://docs.espressif.com/projects/esp-idf/en/latest/api-guides/build-system.html
# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

# Linking the shared components directory to the project
set(EXTRA_COMPONENT_DIRS    ${CMAKE_CURRENT_SOURCE_DIR}/../../components
#                           ${CMAKE_CURRENT_SOURCE_DIR}/../../microros_component
                            )

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(mqtt_power_test)
