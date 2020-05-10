# - Try to find CUDA CUB lib
# Once done this will define
#
#  CUB_FOUND - system has eigen lib with correct version
#  CUB_INCLUDE_DIR - the eigen include directory
#
# This module reads hints about search locations from 
# the following enviroment variables:
#
# CUB_ROOT
# CUB_ROOT_DIR
#
# Redistribution and use is allowed according to the terms of the 3-clause BSD license.

if (CUB_INCLUDE_DIR)

  # in cache already
  set(CUB_FOUND TRUE)

else (CUB_INCLUDE_DIR)

    find_path(CUB_INCLUDE_DIR NAMES cub/cub.cuh
        HINTS
        ENV CUB_ROOT 
        ENV CUB_ROOT_DIR
        PATHS
        ${CMAKE_INSTALL_PREFIX}/include
        ${KDE4_INCLUDE_DIR}
      )

  include(FindPackageHandleStandardArgs)
  find_package_handle_standard_args(CUB DEFAULT_MSG CUB_INCLUDE_DIR)
  mark_as_advanced(CUB_INCLUDE_DIR)
endif(CUB_INCLUDE_DIR)

