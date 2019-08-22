# - Try to find the Google Glog library
#
#  This module defines the following variables
#
#  GLOG_FOUND - Was Glog found
#  GLOG_INCLUDE_DIRS - the Glog include directories
#  GLOG_LIBRARIES - Link to this
#
#  This module accepts the following variables
#
#  GLOG_ROOT - Can be set to Glog install path or Windows build path
#

find_package(PkgConfig REQUIRED)
pkg_check_modules(PC_GLOG REQUIRED libglog)
if(PC_GLOG_FOUND)
    set(GLOG_FOUND TRUE)
    set(GLOG_INCLUDE_DIRS ${PC_GLOG_INCLUDE_DIRS})
    set(GLOG_LIBRARY_DIRS ${PC_GLOG_LIBRARY_DIRS})
    set(GLOG_LIBRARIES ${PC_GLOG_LIBRARIES} -pthread -lgflags)
else()
    set(GLOG_FOUND FALSE)
endif()