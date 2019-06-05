cmake_minimum_required(VERSION 3.10)
project(supereight)

################################################################################
# Add local path for finding packages, set the local version first
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake)

################################################################################

#add_definitions(-DEIGEN_DONT_VECTORIZE)
add_compile_options(-std=c++14 )# -mtune=generic)
add_subdirectory(se_core)
add_subdirectory(se_shared)
add_subdirectory(se_tools)

# Build se_denseslam lib
option(WITH_OPENMP "Compile with OpenMP" ON)


set(BUILT_LIBS "")
add_subdirectory(se_denseslam)
#add_subdirectory(se_apps)


################################################################################
