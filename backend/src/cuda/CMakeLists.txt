cmake_minimum_required(VERSION 3.10)

project(supereight-backend)

enable_language(CUDA)

find_package(CUDAToolkit REQUIRED)
find_package(Eigen3      REQUIRED)
find_package(Sophus      REQUIRED)

set(libraries CUDA::cudart supereight-shared supereight-core)

if(SE_OPENMP)
    list(APPEND compile_flags ${OpenMP_CXX_FLAGS})
    list(APPEND libraries ${OpenMP_CXX_FLAGS})
endif()

foreach(FIELD_TYPE ${FIELD_TYPES})
    string(TOLOWER ${FIELD_TYPE} FIELD_TYPE_NAME)

    set(library_name ${PROJECT_NAME}-cuda-${FIELD_TYPE_NAME})

    add_library(${library_name} STATIC backend_cuda.cpp)

    target_include_directories(${library_name} PUBLIC ${BACKEND_INCLUDE_DIR}
        ${TOON_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIR} ${SOPHUS_INCLUDE_DIR})
    target_compile_options(${library_name} PUBLIC ${compile_flags})
    target_link_libraries(${library_name} ${libraries})

    target_compile_definitions(${library_name} PUBLIC SE_CUDA_VF
        SE_BACKEND=cuda SE_FIELD_TYPE=${FIELD_TYPE})

    list(APPEND BUILT_BACKENDS cuda-${FIELD_TYPE_NAME})
endforeach()

set(BUILT_BACKENDS ${BUILT_BACKENDS} PARENT_SCOPE)
