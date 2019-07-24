cmake_minimum_required(VERSION 2.8)

set(PROJECT_TEST_NAME PinholeCamera-unittest)

set(UNIT_TEST_NAME PinholeCamera-unittest)
set(UNIT_TEST_SRC
  "../../src/camera.cpp"
  "PinholeCamera_unittest.cpp")

add_executable(${UNIT_TEST_NAME} ${UNIT_TEST_SRC})
target_include_directories(${UNIT_TEST_NAME} PUBLIC ${GTEST_INCLUDE_DIRS})
target_link_libraries(${UNIT_TEST_NAME} ${GTEST_BOTH_LIBRARIES} pthread)

GTEST_ADD_TESTS(${UNIT_TEST_NAME} "" AUTO)

