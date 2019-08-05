cmake_minimum_required(VERSION 3.10)

# Define the library targets. Use generator expressions (BUILD_INTERFACE and
# INSTALL_INTERFACE) to change the include directories when building the
## library and when it is installed and used from another project.
## Header only library containing headers common for all supereight modules.
#add_library(se-ompl INTERFACE)
#target_include_directories(se-ompl INTERFACE
#  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#  $<INSTALL_INTERFACE:include>)
#install(
#  TARGETS se-ompl
#  EXPORT supereightTargets)
## Populate the list of libraries that are part of se-shared.
#list(APPEND SUPEREIGHT_OMPL_LIBS se-ompl)



add_library(se-vs STATIC
        src/volume_shifter.cpp
        src/utils/timing.cpp)

target_link_libraries(se-vs  PUBLIC
        ${SUPEREIGHT_CORE_LIBS}
        )

target_include_directories(se-vs PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        ${EIGEN3_INCLUDE_DIR}
        )

install(
        TARGETS se-vs
        EXPORT supereightTargets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include)
# Populate the list of libraries that are part of se-shared.
list(APPEND SUPEREIGHT_OMPL_LIBS se-vs)

install(
        DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/
        DESTINATION include)



# Make the list of se-ompl libraries available to the parent file.
set(SUPEREIGHT_OMPL_LIBS ${SUPEREIGHT_OMPL_LIBS} PARENT_SCOPE)
