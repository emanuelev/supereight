cmake_minimum_required(VERSION 3.10)
project(octree_lib)


set(PROJECT_TEST_NAME utils)
set(UNIT_TEST_NAME ${PROJECT_TEST_NAME}-morton-unittest)
add_executable(${UNIT_TEST_NAME} morton_unittest.cpp)
target_include_directories(${UNIT_TEST_NAME} PUBLIC ${GTEST_INCLUDE_DIRS})
target_link_libraries(${UNIT_TEST_NAME} ${GTEST_BOTH_LIBRARIES} pthread)

GTEST_ADD_TESTS(${UNIT_TEST_NAME} "" AUTO)

