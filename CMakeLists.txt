cmake_minimum_required(VERSION 3.10)
project(supereight)


set( CMAKE_VERBOSE_MAKEFILE ON)
################################################################################
# Add local path for finding packages, set the local version first
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})

################################################################################
# Create variables used for exporting (from build tree) in supereightConfig.cmake
set( supereight_LIBRARIES "" )
set( supereight_INCLUDE_DIR ${PROJECT_SOURCE_DIR} )

################################################################################
add_compile_options(-std=c++14 -mtune=generic)

#find_package(catkin REQUIRED COMPONENTS roscpp rospy)


# including subprojects
add_subdirectory(se_core)
add_subdirectory(se_shared)
add_subdirectory(se_tools)

# Build se_denseslam lib
option(WITH_OPENMP "Compile with OpenMP" ON)


set(BUILT_LIBS "")
add_subdirectory(se_denseslam)
add_subdirectory(se_apps)


################################################################################
# Export package for use from the build tree
EXPORT( PACKAGE supereight )

# Create the supereightConfig.cmake file for other cmake projects.
# ... for the build tree
SET( CONFIG_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR})
SET( CONFIG_DIR ${CMAKE_CURRENT_BINARY_DIR})

CONFIGURE_FILE( ${CMAKE_CURRENT_SOURCE_DIR}/supereightConfig.cmake.in
        ${CMAKE_CURRENT_BINARY_DIR}/supereightConfig.cmake @ONLY )
# ... for the install tree
SET( CMAKECONFIG_INSTALL_DIR /usr/local/lib/cmake/supereight )
FILE( RELATIVE_PATH REL_INCLUDE_DIR
        "${CMAKE_INSTALL_PREFIX}/${CMAKECONFIG_INSTALL_DIR}"
        "${CMAKE_INSTALL_PREFIX}/include" )

SET( supereight_INCLUDE_DIR "/usr/local/include/se" )
SET( CONFIG_SOURCE_DIR )
SET( CONFIG_DIR )

CONFIGURE_FILE( ${CMAKE_CURRENT_SOURCE_DIR}/supereightConfig.cmake.in
        ${CMAKE_CURRENT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/supereightConfig.cmake @ONLY )


INSTALL(FILES "${CMAKE_CURRENT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/supereightConfig.cmake"
        DESTINATION ${CMAKECONFIG_INSTALL_DIR})

