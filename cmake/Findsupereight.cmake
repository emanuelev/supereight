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

if (SUPEREIGHT_INCLUDE_DIR)

    # in cache already
    set(SUPEREIGHT_FOUND TRUE)

else (SUPEREIGHT_INCLUDE_DIR)

    find_path(SUPEREIGHT_INCLUDE_DIR
            NAMES se
            HINTS
            ENV SUPEREIGHT_ROOT
            ENV SUPEREIGHT_ROOT_DIR
            PATHS
            ${CMAKE_INSTALL_PREFIX}/include
            ${KDE4_INCLUDE_DIR}
            PATH_SUFIXES supereight
            )

    include(FindPackageHandleStandardArgs)
    find_package_handle_standard_args(SUPEREIGHT DEFAULT_MSG SUPEREIGHT_INCLUDE_DIR)
    mark_as_advanced(SUPEREIGHT_INCLUDE_DIR)
endif(SUPEREIGHT_INCLUDE_DIR)
