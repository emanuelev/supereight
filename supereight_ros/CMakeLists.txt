cmake_minimum_required(VERSION 3.10)
project(supereight_ros)

add_compile_options(-std=c++11)

add_definitions(-DEIGEN_MAX_ALIGN_BYTES=16)

message(WARNING "CMAKE_CXX_FLAGS_DEBUG: " ${CMAKE_CXX_FLAGS_DEBUG})
message(WARNING "CMAKE_CXX_FLAGS_RELEASE: " ${CMAKE_CXX_FLAGS_RELEASE})

#set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} /usr/local/lib/cmake)
#set(APPEND $CMAKE_PREFIX_PATH "/usr/local" )
find_package(Eigen3 REQUIRED)
find_package(supereight REQUIRED)
## find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  message_filters
  rosbag
  roscpp
  rospy
  std_msgs
  geometry_msgs
      )



#############
## Catkin ##
#############
## catkin specific configuration
# declare things to be passed to dependent projects
# supereight_ros_node and tests
catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME}
        CATKIN_DEPENDS roscpp std_msgs rospy geometry_msgs message_filters
        rosbag
        cv_bridge
)

#############
## Build ##
#############


## Specify additional locations of header files
## your package locations should be listed before other locations
## all targets in this CMakeList will have the directories added to their
## include path
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${supereight_INCLUDE_DIR}
        ${EIGEN3_INCLUDE_DIR}
)



add_executable(supereight_ros_node
        src/supereight_ros_node.cpp
        src/supereight_ros.cpp
        )


## Used for supereight package
# Options for SE_FIELD_TYPE: OFusion, Sdf
set(field_type SE_FIELD_TYPE=OFusion)
target_compile_definitions(supereight_ros_node PUBLIC ${field_type})

target_link_libraries(supereight_ros_node
        ${catkin_LIBRARIES} )


#############
## Testing ##
#############

# Add gtest based cpp test target and link libraries
catkin_add_gtest(supereight_ros-test
        test/main.cpp
        test/test_example.cpp
)

target_compile_definitions(supereight_ros-test PUBLIC ${field_type})
target_link_libraries(supereight_ros-test
        ${CMAKE_THREAD_LIBS_INIT}
        ${catkin_LIBRARIES})

##
#find_package(GLUT)
#find_package(OPENGL)
#set(additional_libraries stdc++ rt GL ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES})
#add_executable(${PROJECT_NAME}_node src/supereight_ros_node.cpp)
#target_include_directories(supereight_ros_node PUBLIC ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})
#target_link_libraries(supereight_ros_node ${catkin_LIBRARIES} se-denseslam-ofusion ${additional_libraries})
