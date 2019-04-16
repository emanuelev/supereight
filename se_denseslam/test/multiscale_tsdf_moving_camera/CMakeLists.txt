cmake_minimum_required(VERSION 2.8)
project(se_denseslam)

set(PROJECT_TEST_NAME multiscale_integration)
add_executable(${PROJECT_TEST_NAME} tsdf_moving_camera_unittest.cpp)
target_include_directories(${PROJECT_TEST_NAME} PUBLIC ${GTEST_INCLUDE_DIRS})
target_link_libraries(${PROJECT_TEST_NAME} ${GTEST_BOTH_LIBRARIES} pthread)

GTEST_ADD_TESTS(${PROJECT_TEST_NAME} "" AUTO)
