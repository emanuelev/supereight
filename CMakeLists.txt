cmake_minimum_required(VERSION 3.10)

project(supereight)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake)

add_compile_options(-std=c++17 -march=native -fdiagnostics-color=always
  -Wall -Wextra -pedantic -Wno-unknown-pragmas)
add_subdirectory(core)
add_subdirectory(shared)
add_subdirectory(tools)

# Build se_denseslam lib
option(WITH_OPENMP "Compile with OpenMP" ON)

add_subdirectory(tracking)
add_subdirectory(backend)

set(BUILT_LIBS "")
add_subdirectory(denseslam)
add_subdirectory(apps)

