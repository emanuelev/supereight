cmake_minimum_required(VERSION 3.10)

project(supereight-denseslam)

find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)

set(libraries lodepng supereight-shared supereight-core
    supereight-tracking)

if (SE_OPENMP)
    list(APPEND compile_flags ${OpenMP_CXX_FLAGS})
    list(APPEND libraries ${OpenMP_CXX_FLAGS})
endif()

foreach(backend ${BUILT_BACKENDS})
    set(library_name ${PROJECT_NAME}-${backend})
    add_library(${library_name} STATIC ./src/DenseSLAMSystem.cpp)

    target_include_directories(${library_name} PUBLIC include
        ${TOON_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIR} ${SOPHUS_INCLUDE_DIR})
    target_compile_options(${library_name} PUBLIC ${compile_flags})
    target_link_libraries(${library_name} ${libraries} supereight-backend-${backend})

    list(APPEND BUILT_LIBS ${library_name})
endforeach()

set(BUILT_LIBS ${BUILT_LIBS} PARENT_SCOPE)
