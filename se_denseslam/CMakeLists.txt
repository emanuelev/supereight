cmake_minimum_required(VERSION 3.10)

project(se-denseslam)

if(APPLE)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -D__APPLE__")
endif(APPLE)

find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)
find_package(OpenCV REQUIRED)

if(WITH_OPENMP)
  find_package(OpenMP)
  if(OPENMP_FOUND)
    message(STATUS "Compiling with OpenMP support")
  endif()
endif()

set(appname ${PROJECT_NAME})


# ---- PREPARE COMMON DEPENDENCIES  ------------ 
set(compile_flags  -Wall -Wextra -Wno-unknown-pragmas)
set(libraries  ${OpenCV_LIBS} lodepng se_shared se_core)

if (WITH_OPENMP AND OPENMP_FOUND)
    list(APPEND compile_flags ${OpenMP_CXX_FLAGS})
    list(APPEND libraries ${OpenMP_CXX_FLAGS})
endif()

# ----------------- OFusion -----------------
set(field_type SE_FIELD_TYPE=OFusion)

add_library(${appname}-ofusion STATIC ./src/DenseSLAMSystem.cpp)
target_include_directories(${appname}-ofusion PUBLIC include
    ${TOON_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIR} ${SOPHUS_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS})
target_compile_options(${appname}-ofusion PUBLIC ${compile_flags})
target_link_libraries(${appname}-ofusion ${libraries})
target_compile_definitions(${appname}-ofusion PUBLIC ${field_type})

list(APPEND BUILT_LIBS ${appname}-ofusion)

# ----------------- SDF -----------------
#set(field_type SE_FIELD_TYPE=SDF)
#
#add_library(${appname}-sdf  ./src/DenseSLAMSystem.cpp)
#target_include_directories(${appname}-sdf PUBLIC include
#    ${TOON_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIR} ${SOPHUS_INCLUDE_DIR})
#target_compile_options(${appname}-sdf PUBLIC ${compile_flags})
#target_link_libraries(${appname}-sdf ${libraries})
#target_compile_definitions(${appname}-sdf PUBLIC ${field_type})
#
#list(APPEND BUILT_LIBS ${appname}-sdf)
#
set(BUILT_LIBS ${BUILT_LIBS} PARENT_SCOPE)


# --------------- INSTALL -----------------------
install(
        TARGETS ${BUILT_LIBS}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
)

install(
        DIRECTORY include/
        DESTINATION include
        FILES_MATCHING PATTERN "*.h*"
)
