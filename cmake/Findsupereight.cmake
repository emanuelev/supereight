# - Try to find Eigen3 lib
# Once done this will define
#
#  SUPEREIGHT_FOUND - system has eigen lib with correct version
#  SUPEREIGHT_INCLUDE_DIR - the eigen include directory
#
# This module reads hints about search locations from
# the following enviroment variables:
#
# SUPEREIGHT_ROOT
# SUPEREIGHT_ROOT_DIR
#
# Redistribution and use is allowed according to the terms of the 3-clause BSD license.

if (SUPEREIGHT_INCLUDE_DIRS)

    # in cache already
    set(SUPEREIGHT_FOUND TRUE)

else (SUPEREIGHT_INCLUDE_DIRS)

    find_path(SUPEREIGHT_INCLUDE_DIRS
            NAMES se
            HINTS
            ENV SUPEREIGHT_ROOT
            ENV SUPEREIGHT_ROOT_DIR
            PATHS
            ${CMAKE_INSTALL_PREFIX}/include
            )

    find_library(SUPEREIGHT_LIBRARIES
            NAMES
                se-denseslam
            PATHS
                ${CMAKE_INSTALL_PREFIX}/lib
    )

    include(FindPackageHandleStandardArgs)

    find_package_handle_standard_args(
            SUPEREIGHT DEFAULT_MSG
            SUPEREIGHT_INCLUDE_DIRS)

    mark_as_advanced(SUPEREIGHT_INCLUDE_DIRS)
endif(SUPEREIGHT_INCLUDE_DIRS)
