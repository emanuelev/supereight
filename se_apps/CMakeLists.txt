cmake_minimum_required(VERSION 3.10)

project(se-apps)



# Fucntion to create the executables given the name of one of the compiled
# se-denseslam libraries.
function(add_version lib_name libraries)

  if(APPLE)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -D__APPLE__")
    set(main_common_libraries
      ${libraries})
  else(APPLE)
    set(main_common_libraries
      ${libraries}
      rt)
  endif(APPLE)

  set(main_common_compile_flags "")

  if (OPENNI2_FOUND)
    include_directories(${OPENNI2_INCLUDE_PATHS})
    set(main_common_libraries
      ${main_common_libraries}
      ${OPENNI2_LIBRARIES})
    set(main_common_compile_flags
      "-DDO_OPENNI ${main_common_compile_flags}")
  endif(OPENNI2_FOUND)

  #  ------ BENCHMARK --------
  add_executable(${lib_name}-benchmark
    src/benchmark.cpp
    src/PowerMonitor.cpp)
  target_link_libraries(${lib_name}-benchmark
    ${lib_name}
    ${main_common_libraries})
  target_include_directories(${lib_name}-benchmark PUBLIC
    include)

  #  ------ MAIN --------
  if (GLUT_FOUND)
    add_executable(${lib_name}-main
      src/mainQt.cpp
      src/PowerMonitor.cpp
      src/reader.cpp)
    target_include_directories(${lib_name}-main PUBLIC
      include)

    if(APPLE)
      target_link_libraries(${lib_name}-main
        ${lib_name}
        ${GLUT_LIBRARIES}
        ${OPENGL_LIBRARIES}
        ${main_common_libraries})
    else(APPLE)
      target_link_libraries(${lib_name}-main
        ${lib_name}
        ${GLUT_LIBRARIES}
        ${OPENGL_LIBRARIES}
        ${main_common_libraries}
        GL)
    endif(APPLE)
  endif(GLUT_FOUND)

  #  ------ QT --------
  if (Qt5OpenGL_FOUND)

    file(GLOB qt_srcs qt/*)

    QT5_ADD_RESOURCES( ResourceSources
      "qt/images.qrc")
    set(qt_srcs
      ${qt_srcs}
      ${ResourceSources})
    set(CMAKE_SHARED_LIBRARY_LINK_CXX_FLAGS "")
    set(CMAKE_SHARED_LIBRARY_LINK_C_FLAGS "")
    set(CMAKE_AUTOMOC ON)
    set(CMAKE_INCLUDE_CURRENT_DIR ON)

    include_directories(qt)
    add_definitions(-D_FILE_OFFSET_BITS=64 -fPIC)

    add_executable(${lib_name}-qt
      src/mainQt.cpp
      src/PowerMonitor.cpp
      ${qt_srcs}
      src/reader.cpp)
    target_include_directories(${lib_name}-qt PUBLIC
      include)

    add_definitions(${Qt5Widgets_DEFINITIONS})
    add_definitions(${Qt5OpenGL_DEFINITIONS})
    add_definitions(${Qt5PrintSuppoer_DEFINITIONS})
    include_directories(${Qt5Widgets_INCLUDE_DIRS})
    include_directories(${Qt5OpenGL_INCLUDE_DIRS})
    include_directories(${Qt5PrintSupport_INCLUDE_DIRS})
    include_directories(${Qt5Gui_INCLUDE_DIRS})

    SET_TARGET_PROPERTIES(${lib_name}-qt PROPERTIES COMPILE_FLAGS
      "${main_common_compile_flags} -D__QT__")

    if(APPLE)
      target_link_libraries(${lib_name}-qt
        Qt5::Core
        Qt5::Gui
        Qt5::Widgets
        Qt5::OpenGL
        Qt5::PrintSupport
        ${main_common_libraries}
        ${lib_name}
        ${GLUT_LIBRARIES}
        ${OPENGL_LIBRARIES})
    else(APPLE)
      target_link_libraries(${lib_name}-qt
        Qt5::Core
        Qt5::Gui
        Qt5::Widgets
        Qt5::OpenGL
        Qt5::PrintSupport
        ${main_common_libraries}
        ${lib_name}
        ${GLUT_LIBRARIES}
        ${OPENGL_LIBRARIES}
        GL)
    endif(APPLE)

  endif(Qt5OpenGL_FOUND)

endfunction()



# Find optional packages.
find_package(GLUT)
find_package(OpenGL)
find_package(OpenNI2)
find_package(PkgConfig)
find_package(Qt5Core)
find_package(Qt5Widgets)
find_package(Qt5Gui)
find_package(Qt5OpenGL)
find_package(Qt5PrintSupport)

# Set the common libraries to link.
set(common_libraries stdc++ ${SUPEREIGHT_CORE_LIBS} ${SUPEREIGHT_OMPL_LIBS})

# Create the executables for each of the compiled se-denseslam libraries.
foreach(lib_name ${SUPEREIGHT_DENSESLAM_LIBS})
  add_version(${lib_name} "${common_libraries}")
endforeach()

