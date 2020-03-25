cmake_minimum_required(VERSION 3.0)

project(StereoCameraDemo)

include(../common.cmake)

add_executable(StereoCameraDemo
    main.cpp mycamerahandler.cpp
)

set_target_properties(StereoCameraDemo
    PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY "${BIN_DIR}"
    LIBRARY_OUTPUT_DIRECTORY "${BIN_DIR}"
    RUNTIME_OUTPUT_DIRECTORY "${BIN_DIR}"
)
