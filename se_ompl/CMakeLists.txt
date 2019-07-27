cmake_minimum_required(VERSION 3.10)



# Define the library targets. Use generator expressions (BUILD_INTERFACE and
# INSTALL_INTERFACE) to change the include directories when building the
# library and when it is installed and used from another project.

set(LIB_SOURCE_FILES
        src/occupancy_world.cpp
        src/volume_shifter.cpp
        )

add_library(se-ompl SHARED
        ${LIB_SOURCE_FILES} )
target_link_libraries(se-ompl  PUBLIC ${OMPL_LIBRARIES}
        ${GLOG_LIBRARIES}  ${YAMLCPP_LIBRARY})
target_include_directories(se-ompl PUBLIC
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


# Make the list of se-ompl libraries available to the parent file.
set(SUPEREIGHT_OMPL_LIBS ${SUPEREIGHT_OMPL_LIBS} PARENT_SCOPE)
