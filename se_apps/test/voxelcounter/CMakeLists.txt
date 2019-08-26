cmake_minimum_required(VERSION 2.8)

set(PROJECT_TEST_NAME voxelcounter_unittest)



set(UNIT_TEST_NAME voxelcounter-unittest)
add_definitions(-DASTAR_PLANNER)
add_executable(${UNIT_TEST_NAME} voxelcounter_unittest.cpp)
target_include_directories(${UNIT_TEST_NAME} PUBLIC ${GTEST_INCLUDE_DIRS})
target_link_libraries(${UNIT_TEST_NAME} ${SUPEREIGHT_PATH_PLANNING_LIBS} ${GTEST_BOTH_LIBRARIES} pthread)

GTEST_ADD_TESTS(${UNIT_TEST_NAME} "" AUTO)

