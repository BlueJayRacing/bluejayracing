# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

set(EXTRA_COMPONENT_DIRS    ${CMAKE_CURRENT_SOURCE_DIR}/../../components
                            )

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
<<<<<<<< HEAD:car/firmware/esp_idf/prod/04-in-shaft-wireless-strain-gauge/CMakeLists.txt
project(04-in-shaft-wireless-strain-gauge)
========
project(01-wireless-strain-gauge-cal)
>>>>>>>> main:car/firmware/esp_idf/prod/01-wireless-strain-gauge-cal/CMakeLists.txt
