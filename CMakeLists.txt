cmake_minimum_required(VERSION 3.10)

project(supereight)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake)

add_compile_options(-std=c++17 -march=native -fdiagnostics-color=always
  -Wall -Wextra -pedantic -Wno-unknown-pragmas)

find_package(OpenMP)

option(SE_BACKEND_CPU    "Build CPU backend"    ON)
option(SE_BACKEND_OPENMP "Build OpenMP backend" ${OPENMP_FOUND})

option(SE_OPENMP "Build preprocessing and tracking with OpenMP support" ${OPENMP_FOUND})

add_subdirectory(core)
add_subdirectory(shared)
add_subdirectory(tools)
add_subdirectory(tracking)

set(BUILT_BACKENDS "")
add_subdirectory(backend)

message(STATUS "BUILT_BACKENDS=${BUILT_BACKENDS}")

set(BUILT_LIBS "")
add_subdirectory(denseslam)

message(STATUS "BUILT_LIBS=${BUILT_LIBS}")

add_subdirectory(apps)

