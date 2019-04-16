cmake_minimum_required(VERSION 3.10)

add_library(se_shared INTERFACE)
target_include_directories(se_shared INTERFACE thirparty .)

install(
        DIRECTORY .
        DESTINATION include
        FILES_MATCHING PATTERN "*.h*"
)

