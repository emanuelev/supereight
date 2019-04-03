cmake_minimum_required(VERSION 2.8)
project(se_denseslam)

find_package(OpenCV REQUIRED)

IF(OpenCV_FOUND)
    message(STATUS "Found OpenCV_INCLUDE_DIRS at ${OpenCV_INCLUDE_DIRS}")
    message(STATUS "Found OpenCV_INCLUDE_DIRS at ${OpenCV_LIBRARIES}")
    message(STATUS "Found OpenCV_INCLUDE_DIRS at ${OpenCV_LIBS}")
ENDIF(OpenCV_FOUND)

set(PROJECT_TEST_NAME multiscale_integration)
add_executable(${PROJECT_TEST_NAME} ../multiscale_integration/integration_unittest.cpp)
target_include_directories(${PROJECT_TEST_NAME} PUBLIC ${GTEST_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
target_link_libraries(${PROJECT_TEST_NAME} ${GTEST_BOTH_LIBRARIES} ${OpenCV_LIBS} pthread)

GTEST_ADD_TESTS(${PROJECT_TEST_NAME} "" AUTO)
