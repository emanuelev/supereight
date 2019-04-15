cmake_minimum_required(VERSION 3.10)

add_library(se_core INTERFACE )
target_include_directories(se_core INTERFACE include)


INSTALL(
        DIRECTORY include/
        DESTINATION /usr/local/include
        FILES_MATCHING PATTERN "*.h*"
)