cmake_minimum_required(VERSION 3.10)

add_library(shared INTERFACE)
target_include_directories(shared INTERFACE thirparty .)
