cmake_minimum_required(VERSION 3.10)
project(octree_lib)


set(PROJECT_TEST_NAME algorithms_unittest)
add_executable(${PROJECT_TEST_NAME} unique_unittest.cpp)
target_include_directories(${PROJECT_TEST_NAME} PUBLIC ${GTEST_INCLUDE_DIRS})
target_link_libraries(${PROJECT_TEST_NAME} ${GTEST_BOTH_LIBRARIES} pthread)

GTEST_ADD_TESTS(${PROJECT_TEST_NAME} "" AUTO)
