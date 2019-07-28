cmake_minimum_required(VERSION 3.10)

# Define the library targets. Use generator expressions (BUILD_INTERFACE and
# INSTALL_INTERFACE) to change the include directories when building the
# library and when it is installed and used from another project.
find_package(glog REQUIRED)
find_package(ompl REQUIRED)
find_package(yaml-cpp REQUIRED)
set(LIB_SOURCE_FILES
#        src/occupancy_world.cpp
        src/volume_shifter.cpp
        )

add_library(se-ompl STATIC
        src/volume_shifter.cpp)
target_link_libraries(se-ompl  INTERFACE ${OMPL_LIBRARIES}
        ${GLOG_LIBRARIES}  ${YAML_CPP_LIBRARIES})
target_include_directories(se-ompl INTERFACE
        ${OMPL_INCLUDE_DIRS}
        ${GLOG_INCLUDE_DIRS}
        ${YAML_CPP_INCLUDE_DIR}
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
install(
        TARGETS se-ompl
        EXPORT supereightTargets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include)
# Populate the list of libraries that are part of se-shared.
list(APPEND SUPEREIGHT_OMPL_LIBS se-ompl)

install(
        DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/
        DESTINATION include)

# When linking the se-core library, the se-shared libraries should also be
# linked. Append them to the list.
list(APPEND SUPEREIGHT_OMPL_LIBS ${SUPEREIGHT_SHARED_LIBS})

# Make the list of se-ompl libraries available to the parent file.
set(SUPEREIGHT_OMPL_LIBS ${SUPEREIGHT_OMPL_LIBS} PARENT_SCOPE)
