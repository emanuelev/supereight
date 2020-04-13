cmake_minimum_required(VERSION 3.10)

project(supereight-tracking)

find_package(Eigen3 REQUIRED)

if(WITH_OPENMP)
  find_package(OpenMP)
  if(OPENMP_FOUND)
    message(STATUS "Compiling with OpenMP support")
  endif()
endif()

set(appname ${PROJECT_NAME})

set(libraries supereight-shared supereight-core)

if (WITH_OPENMP AND OPENMP_FOUND)
    list(APPEND compile_flags ${OpenMP_CXX_FLAGS})
    list(APPEND libraries ${OpenMP_CXX_FLAGS})
endif()

add_library(${appname} ./src/tracker.cpp)
target_include_directories(${appname} PUBLIC include
    ${TOON_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIR})

target_compile_options(${appname} PUBLIC ${compile_flags})
target_link_libraries(${appname} ${libraries})
