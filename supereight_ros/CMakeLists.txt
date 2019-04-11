cmake_minimum_required(VERSION 3.10)
project(supereight_ros)

add_compile_options(-std=c++14  -march=native)

add_definitions(-DEIGEN_MAX_ALIGN_BYTES=16)

message(WARNING "CMAKE_CXX_FLAGS_DEBUG: " ${CMAKE_CXX_FLAGS_DEBUG})
message(WARNING "CMAKE_CXX_FLAGS_RELEASE: " ${CMAKE_CXX_FLAGS_RELEASE})

option(WITH_OPENMP "Compile with OpenMP" ON)
find_package(Eigen3 REQUIRED)
find_package(supereight REQUIRED)
## find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  message_filters
  rosbag
  roscpp
  roslib
  std_msgs
  message_generation visualization_msgs geometry_msgs

      )

add_message_files(FILES
  ImagePose.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)

if(WITH_OPENMP)
    find_package(OpenMP)
    if(OPENMP_FOUND)
        message(STATUS "Compiling with OpenMP support")
    endif()
endif()

if (WITH_OPENMP AND OPENMP_FOUND)
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

#############
## Catkin ##
#############
## catkin specific configuration
# declare things to be passed to dependent projects
# supereight_ros_node and tests
catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME}
        CATKIN_DEPENDS
        roscpp
        rosbag
        geometry_msgs
        std_msgs
        message_runtime
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
        src/pipeline_support_functions.cpp
        )


## Used for supereight package
# Options for SE_FIELD_TYPE: OFusion, Sdf
set(field_type SE_FIELD_TYPE=OFusion)
target_compile_definitions(supereight_ros_node PUBLIC ${field_type})

target_link_libraries(supereight_ros_node
        se-denseslam-ofusion
        lodepng
        ${catkin_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})

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
