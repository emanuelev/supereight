cmake_minimum_required(VERSION 3.10)
project(voxelcounter_unit_testing)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR}/cmake)

# GTest Root - Change to reflect your install dir
set(GTEST_ROOT ~/Documents/Programs/Source/googletest/googletest)
find_package(GTest REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)

enable_testing()
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
add_compile_options(-std=c++14)
include_directories(../include/se ../include ../../se_core/include/se ../../se_core/include ../../se_denseslam/include/se ../../se_denseslam/include ../../se_shared/include/se ../../se_shared/include ${EIGEN3_INCLUDE_DIR} ${SOPHUS_INCLUDE_DIR})

add_subdirectory(voxelcounter)

