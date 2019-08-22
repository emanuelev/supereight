cmake_minimum_required(VERSION 3.10)

find_package(ompl REQUIRED)
# Define the library targets. Use generator expressions (BUILD_INTERFACE and
# INSTALL_INTERFACE) to change the include directories when building the
# library and when it is installed and used from another project.

# Header only library containing headers common for all supereight modules.
add_library(se-shared INTERFACE)
target_include_directories(se-shared INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
install(
  TARGETS se-shared
  EXPORT supereightTargets)
# Populate the list of libraries that are part of se-shared.
list(APPEND SUPEREIGHT_SHARED_LIBS se-shared)



# lodepng third-party shared library.
add_library(lodepng SHARED src/lodepng.cpp)
target_include_directories(lodepng PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
install(
  TARGETS lodepng
  EXPORT supereightTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)
# Populate the list of libraries that are part of se-shared.
list(APPEND SUPEREIGHT_SHARED_LIBS lodepng)

add_executable(ompl-demo src/StateSampling.cpp)

target_include_directories(ompl-demo PUBLIC ${OMPL_INCLUDE_DIRS})
target_link_libraries(ompl-demo
        ${OMPL_LIBRARIES}
)


# Install the headers for all libraries in se-shared.
install(
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/
  DESTINATION include)

# Make the list of se-shared libraries available to the parent file.
set(SUPEREIGHT_SHARED_LIBS ${SUPEREIGHT_SHARED_LIBS} PARENT_SCOPE)

