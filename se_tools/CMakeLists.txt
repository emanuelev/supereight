cmake_minimum_required(VERSION 3.10)

if (OPENNI2_FOUND)
  include_directories(${OPENNI2_INCLUDE_PATHS})
  add_executable(oni2raw oni2raw.cpp)
  target_link_libraries(oni2raw ${OPENNI2_LIBRARIES})
endif(OPENNI2_FOUND)

add_executable(scene2raw scene2raw.cpp)
target_link_libraries(scene2raw lodepng)
target_include_directories(scene2raw PUBLIC .)
