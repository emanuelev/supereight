cmake_minimum_required(VERSION 3.10)

add_library(se_shared INTERFACE)
target_include_directories(se_shared INTERFACE include)

add_library(lodepng SHARED src/lodepng.cpp)
target_include_directories(lodepng PUBLIC include)

