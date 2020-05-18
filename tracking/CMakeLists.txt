cmake_minimum_required(VERSION 3.10)

project(supereight-tracking)

set(TRACKING_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_subdirectory(src/openmp)
add_subdirectory(src/cuda)
