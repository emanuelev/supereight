cmake_minimum_required(VERSION 2.8)

set(PROJECT_TEST_NAME interpolation_unittest)

set(UNIT_TEST_NAME gather-unittest)
add_executable(${UNIT_TEST_NAME} gather_unittest.cpp)
target_include_directories(${UNIT_TEST_NAME} PUBLIC ${GTEST_INCLUDE_DIRS})
target_link_libraries(${UNIT_TEST_NAME} ${GTEST_BOTH_LIBRARIES} pthread)

GTEST_ADD_TESTS(${UNIT_TEST_NAME} "" AUTO)

set(UNIT_TEST_NAME interpolation-unittest)
add_executable(${UNIT_TEST_NAME} interpolation_unittest.cpp)
target_include_directories(${UNIT_TEST_NAME} PUBLIC ${GTEST_INCLUDE_DIRS} ../../../apps/kfusion/thirdparty)
target_link_libraries(${UNIT_TEST_NAME} ${GTEST_BOTH_LIBRARIES} pthread)

GTEST_ADD_TESTS(${UNIT_TEST_NAME} "" AUTO)

