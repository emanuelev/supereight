cmake_minimum_required(VERSION 3.10)

add_library(se_shared INTERFACE)
target_include_directories(se_shared INTERFACE thirparty .)

INSTALL(
        DIRECTORY .
        DESTINATION /usr/local/include
        FILES_MATCHING PATTERN "*.h*"
)