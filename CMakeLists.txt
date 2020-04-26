cmake_minimum_required(VERSION 3.10)
include(CheckLanguage)

project(supereight)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -fdiagnostics-color=always")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -pedantic -Wno-unknown-pragmas -Wno-c99-extensions")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native")

set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -std=c++14 -fdiagnostics-color=always -Wall -Wextra -pedantic")

find_package(OpenMP)

check_language(CUDA)
if(CMAKE_CUDA_COMPILER)
    set(CUDA_FOUND ON)
endif()

option(SE_BACKEND_CPU    "Build CPU backend"    ON)
option(SE_BACKEND_OPENMP "Build OpenMP backend" ${OPENMP_FOUND})
option(SE_BACKEND_CUDA   "Build CUDA backend"   ${CUDA_FOUND})

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

