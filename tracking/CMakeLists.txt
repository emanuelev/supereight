cmake_minimum_required(VERSION 3.10)

project(supereight-tracking)

find_package(Eigen3 REQUIRED)

set(libraries supereight-shared supereight-core)

if (SE_OPENMP)
    list(APPEND compile_flags ${OpenMP_CXX_FLAGS})
    list(APPEND libraries ${OpenMP_CXX_FLAGS})
endif()

add_library(${PROJECT_NAME} ./src/tracker.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC include
    ${TOON_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIR})

target_compile_options(${PROJECT_NAME} PUBLIC ${compile_flags})
target_link_libraries(${PROJECT_NAME} ${libraries})
