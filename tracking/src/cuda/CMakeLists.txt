cmake_minimum_required(VERSION 3.10)

project(supereight-tracking)

enable_language(CUDA)

find_package(CUDAToolkit REQUIRED)
find_package(Eigen3 REQUIRED)

set(libraries CUDA::cudart supereight-shared supereight-core)

set(library_name ${PROJECT_NAME}-cuda)

add_library(${library_name} STATIC ./tracker_cuda.cu)

target_include_directories(${library_name} PUBLIC ${TRACKING_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIR})
target_compile_options(${library_name} PUBLIC ${compile_flags})
target_link_libraries(${library_name} ${libraries})

target_compile_definitions(${library_name} PUBLIC SE_CUDA_VF SE_TRACKER=cuda)
