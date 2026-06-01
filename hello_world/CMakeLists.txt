cmake_minimum_required(VERSION 3.22)

# 1. Point explicitly to the exact folder where the component's CMakeLists.txt resides
set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/components/dw3000-decadriver-source/platform/esp-idf/decadriver")

# 2. Exclude the root folder from being automatically evaluated as a broken component
set(COMPONENTS main)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(hello_world)
