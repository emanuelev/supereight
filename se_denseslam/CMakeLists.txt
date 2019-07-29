cmake_minimum_required(VERSION 3.10)

# This project name will also be the prefix of the libraries compiled from
# se_denseslam.
project(se-denseslam)


# Find dependencies.
find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)
find_package(OpenCV REQUIRED)
find_package(glog REQUIRED)
find_package(ompl REQUIRED)
find_package(yaml-cpp REQUIRED)
if (WITH_OPENMP)
    find_package(OpenMP)
    if (OPENMP_FOUND)
        message(STATUS "Compiling with OpenMP support")
    endif ()
endif ()


# Set the compiler flags and linked libraries.
set(compile_flags -Wall -Wextra -Wno-unknown-pragmas)

set(libraries ${SUPEREIGHT_CORE_LIBS} ${SUPEREIGHT_OMPL_LIBS} )
message(WARNING " libraries ${libraries}")
if (APPLE)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -D__APPLE__")
endif (APPLE)
if (WITH_OPENMP AND OPENMP_FOUND)
    list(APPEND compile_flags ${OpenMP_CXX_FLAGS})
    list(APPEND libraries ${OpenMP_CXX_FLAGS})
endif ()


# List of SE_FIELD_TYPE macro values. The elements of this list shuld coincide
# with the names of the structs defined in
# se_denseslam/include/se/volume_traits.cpp. When adding a new field type
# appending it to this list is enough to compile se-denseslam using it.
#set(SUPEREIGHT_FIELD_TYPES SDF OFusion)
set(SUPEREIGHT_FIELD_TYPES OFusion)

# Iterate over each SE_FIELD_TYPE and compile the se-denseslam library for it.
foreach (field_type ${SUPEREIGHT_FIELD_TYPES})
    # Convert the field type name to lowercase for use in filenames.
    string(TOLOWER ${field_type} field_type_lc)

    # Prepare the preprocessor SE_FIELD_TYPE macro.
    set(field_type_macro SE_FIELD_TYPE=${field_type})

    # Prepare the library name.
    set(lib_name "${PROJECT_NAME}-${field_type_lc}")

    # Add the static library.d
    add_library(${lib_name} STATIC
            ./src/DenseSLAMSystem.cpp ./src/camera.cpp)
    target_include_directories(${lib_name} PUBLIC
#            ../se_ompl/include
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:include>
            ${EIGEN3_INCLUDE_DIR}
            ${OMPL_INCLUDE_DIRS}
            ${GLOG_INCLUDE_DIRS}
            ${YAML_CPP_INCLUDE_DIR}
            )
    target_compile_options(${lib_name} PUBLIC ${compile_flags})
    target_link_libraries(${lib_name} ${libraries}
            ${OMPL_LIBRARIES}
            ${GLOG_LIBRARIES}
            ${YAML_CPP_LIBRARIES} )
    target_compile_definitions(${lib_name} PUBLIC ${field_type_macro})

    # Install the library and headers.
    install(
            TARGETS ${lib_name}
            EXPORT supereightTargets
            LIBRARY DESTINATION lib
            ARCHIVE DESTINATION lib
            RUNTIME DESTINATION bin
            INCLUDES DESTINATION include)
    install(
            DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/
            DESTINATION include)

    # Populate the list of libraries that are part of se-denseslam.
    list(APPEND SUPEREIGHT_DENSESLAM_LIBS ${lib_name} )
endforeach ()


# Make the list of field types available to the parent file.
set(SUPEREIGHT_FIELD_TYPES ${SUPEREIGHT_FIELD_TYPES} PARENT_SCOPE)

# Make the list of compiled se-denseslam libraries available to the parent file.
set(SUPEREIGHT_DENSESLAM_LIBS ${SUPEREIGHT_DENSESLAM_LIBS} PARENT_SCOPE)

