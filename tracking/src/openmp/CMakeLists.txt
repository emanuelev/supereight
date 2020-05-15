cmake_minimum_required(VERSION 3.10)

project(supereight-tracking)

find_package(Eigen3 REQUIRED)

set(libraries supereight-shared supereight-core)

list(APPEND compile_flags ${OpenMP_CXX_FLAGS})
list(APPEND libraries ${OpenMP_CXX_FLAGS})

set(library_name ${PROJECT_NAME}-openmp)

add_library(${library_name} STATIC ./tracker_openmp.cpp)

target_include_directories(${library_name} PUBLIC ${TRACKING_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIR})
target_compile_options(${library_name} PUBLIC ${compile_flags})
target_link_libraries(${library_name} ${libraries})

target_compile_definitions(${library_name} PUBLIC SE_TRACKER=openmp)
