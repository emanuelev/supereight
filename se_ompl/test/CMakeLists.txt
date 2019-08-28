cmake_minimum_required(VERSION 3.10)
project(octree_ompl_lib_unit_testing)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR}/cmake)

# GTest Root - Change to reflect your install dir
set(GTEST_ROOT /home/anna/Software/googletest/googletest)
message(STATUS "google test path " ${GTEST_INCLUDE_DIRS})
find_package(GTest REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)
find_package(glog REQUIRED)
find_package(ompl REQUIRED)

enable_testing()
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
add_compile_options(-std=c++14 -march=native )
add_definitions(-DEIGEN_MAX_ALIGN_BYTES=16)

include_directories(../include/se ../include  ${EIGEN3_INCLUDE_DIR} ${SOPHUS_INCLUDE_DIR} ${GLOG_INCLUDE_DIRS} ${OMPL_INCLUDE_DIRS})

add_subdirectory(collision)

