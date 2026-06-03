# # The following lines of boilerplate have to be in your project's CMakeLists
# # in this exact order for cmake to work correctly
# cmake_minimum_required(VERSION 3.22)

# include($ENV{IDF_PATH}/tools/cmake/project.cmake)
# # "Trim" the build. Include the minimal set of components, main, and anything it depends on.
# idf_build_set_property(MINIMAL_BUILD ON)
# project(espnow_sensor_slave)


# espnow_sensor_slave/CMakeLists.txt
cmake_minimum_required(VERSION 3.22)

# 1. 🎯 Point explicitly to the exact folder where the component's CMakeLists.txt resides relative to slave
set(EXTRA_COMPONENT_DIRS "${CMAKE_SOURCE_DIR}/../hello_world/components/dw3000-decadriver-source/platform/esp-idf/decadriver")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# "Trim" the build. Include the minimal set of components, main, and anything it depends on.
idf_build_set_property(MINIMAL_BUILD ON)
project(espnow_sensor_slave)
