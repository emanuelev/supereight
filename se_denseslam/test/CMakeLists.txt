cmake_minimum_required(VERSION 3.10)
project(octree_lib_unit_testing)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR}/cmake)

# GTest Root - Change to reflect your install dir
set(GTEST_ROOT ~/software/googletest/googletest)
find_package(GTest REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)

enable_testing()
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
add_compile_options(-std=c++14)
include_directories(../../se_core/include 
                    ../include/se ../include 
                    ${EIGEN3_INCLUDE_DIR} 
                    ${SOPHUS_INCLUDE_DIR})

add_subdirectory(multiscale_fusion)
