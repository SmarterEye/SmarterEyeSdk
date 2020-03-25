cmake_minimum_required(VERSION 3.0)

project(GetMotionDataDemo)

include(../common.cmake)

add_executable(GetMotionDataDemo
    main.cpp mycamerahandler.cpp
)

set_target_properties(GetMotionDataDemo
    PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY "${BIN_DIR}"
    LIBRARY_OUTPUT_DIRECTORY "${BIN_DIR}"
    RUNTIME_OUTPUT_DIRECTORY "${BIN_DIR}"
)
