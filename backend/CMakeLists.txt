cmake_minimum_required(VERSION 3.10)

project(supereight-backend)

set(BACKEND_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(BUILT_BACKENDS "")

set(FIELD_TYPES SDF OFusion)

if(SE_BACKEND_OPENMP)
    add_subdirectory(src/openmp)
endif()

if(SE_BACKEND_CUDA)
    add_subdirectory(src/cuda)
endif()

set(BUILT_BACKENDS ${BUILT_BACKENDS} PARENT_SCOPE)
