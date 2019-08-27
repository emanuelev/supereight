cmake_minimum_required(VERSION 3.10)
project(octree_shared_lib)


set(PROJECT_TEST_NAME path_planning)
set(UNIT_TEST_NAME ${PROJECT_TEST_NAME}-candidateview-unittest)
add_executable(${UNIT_TEST_NAME} candidateview_unittest.cpp)
target_include_directories(${UNIT_TEST_NAME} PUBLIC ${GTEST_INCLUDE_DIRS}
         ../../../se_core ../../../se_ompl)
target_link_libraries(${UNIT_TEST_NAME} ${GTEST_BOTH_LIBRARIES} pthread  ${OMPL_LIBRARIES}
            ${GLOG_LIBRARIES})


set(UNIT_TEST_NAME ${PROJECT_TEST_NAME}-planninghistory-unittest)
add_executable(${UNIT_TEST_NAME} planninghistory_unittest.cpp)
target_include_directories(${UNIT_TEST_NAME} PUBLIC ${GTEST_INCLUDE_DIRS}
         ../../../se_core ../../../se_ompl)
target_link_libraries(${UNIT_TEST_NAME} ${GTEST_BOTH_LIBRARIES} pthread  ${OMPL_LIBRARIES}
            ${GLOG_LIBRARIES})

GTEST_ADD_TESTS(${UNIT_TEST_NAME} "" AUTO)


