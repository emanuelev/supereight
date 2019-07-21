# - Try to find Eigen3 lib
# Once done this will define
#
#  SOPHUS_FOUND - system has eigen lib with correct version
#  SOPHUS_INCLUDE_DIR - the eigen include directory
#
# This module reads hints about search locations from 
# the following enviroment variables:
#
# SOPHUS_ROOT
# SOPHUS_ROOT_DIR
#
# Redistribution and use is allowed according to the terms of the 3-clause BSD license.

if (SOPHUS_INCLUDE_DIR)

  # in cache already
  set(SOPHUS_FOUND TRUE)

else (SOPHUS_INCLUDE_DIR)

    find_path(SOPHUS_INCLUDE_DIR NAMES sophus/se3.hpp
        HINTS
        ENV SOPHUS_ROOT 
        ENV SOPHUS_ROOT_DIR
        PATHS
        ${CMAKE_INSTALL_PREFIX}/include
        ${KDE4_INCLUDE_DIR}
      )

  include(FindPackageHandleStandardArgs)
  find_package_handle_standard_args(SOPHUS DEFAULT_MSG SOPHUS_INCLUDE_DIR)
  mark_as_advanced(SOPHUS_INCLUDE_DIR)
endif(SOPHUS_INCLUDE_DIR)

