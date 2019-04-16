cmake_minimum_required(VERSION 3.10)

add_library(se_core INTERFACE )
target_include_directories(se_core INTERFACE include)

install(
        DIRECTORY include/
        DESTINATION include
        FILES_MATCHING PATTERN "*.h*"
)
