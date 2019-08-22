cmake_minimum_required(VERSION 3.10)

project(supereight)
set(SUPEREIGHT_MAJOR_VERSION 0)
set(SUPEREIGHT_MINOR_VERSION 1)
set(SUPEREIGHT_PATCH_VERSION 0)
set(SUPEREIGHT_VERSION
  ${SUPEREIGHT_MAJOR_VERSION}.${SUPEREIGHT_MINOR_VERSION}.${SUPEREIGHT_PATCH_VERSION})

# Add local path for finding packages.
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake)

# Compilation options.
set(CMAKE_CXX_STANDARD 14)
add_compile_options(-march=native)
add_definitions(-DEIGEN_MAX_ALIGN_BYTES=16)
option(WITH_OPENMP "Compile with OpenMP" ON)

# Initialize the variables containing the built libraries and field types.
set(SUPEREIGHT_SHARED_LIBS "")
set(SUPEREIGHT_CORE_LIBS "")
set(SUPEREIGHT_OMPL_LIBS "")
set(SUPEREIGHT_DENSESLAM_LIBS "")
set(SUPEREIGHT_FIELD_TYPES "")

# Build each supereight module.
add_subdirectory(se_shared)
add_subdirectory(se_core)
add_subdirectory(se_ompl)
add_subdirectory(se_denseslam)
add_subdirectory(se_apps)
add_subdirectory(se_tools)



# Generate the required .cmake files for making supereight discoverable by
# CMake after installation.
include(CMakePackageConfigHelpers)

# The directory where .cmake files are installed.
if(WIN32 AND NOT CYGWIN)
  set(INSTALL_CMAKE_DIR cmake)
elseif(APPLE)
  set(INSTALL_CMAKE_DIR ${PROJECT_NAME}.framework/Resources/CMake)
else()
  set(INSTALL_CMAKE_DIR share/${PROJECT_NAME}/cmake)
endif()

# Generate the Version.cmake file.
write_basic_package_version_file(
  "${PROJECT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
  VERSION ${SUPEREIGHT_VERSION}
  COMPATIBILITY AnyNewerVersion)

# Generate the Config.cmake file.
configure_package_config_file(
  "${PROJECT_SOURCE_DIR}/cmake/Config.cmake.in"
  "${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
  INSTALL_DESTINATION ${INSTALL_CMAKE_DIR})

# Install the generated *.cmake files.
install(
  EXPORT supereightTargets
  DESTINATION ${INSTALL_CMAKE_DIR})
install(
  FILES
    "${PROJECT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
    "${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
  DESTINATION ${INSTALL_CMAKE_DIR})



# Create an uninstall target
if(NOT TARGET uninstall)
  configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/cmake_uninstall.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
    IMMEDIATE @ONLY)

  add_custom_target(uninstall
    COMMAND
    ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake)
endif()

