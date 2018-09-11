cmake_minimum_required(VERSION 3.10)

project(se)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake)

add_compile_options(-std=c++14 -march=native)
add_subdirectory(se_core)
add_subdirectory(se_shared)
add_subdirectory(se_tools)

# Build se_denseslam lib
option(WITH_OPENMP "Compile with OpenMP" ON)


set(BUILT_LIBS "")
add_subdirectory(se_denseslam)
add_subdirectory(se_apps)

