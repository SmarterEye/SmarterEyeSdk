cmake_minimum_required(VERSION 3.0)

project(GetCompoundDemo)

include(../common.cmake)

add_executable(GetCompoundDemo
    main.cpp mycamerahandler.cpp
)

set_target_properties(GetCompoundDemo
    PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY "${BIN_DIR}"
    LIBRARY_OUTPUT_DIRECTORY "${BIN_DIR}"
    RUNTIME_OUTPUT_DIRECTORY "${BIN_DIR}"
)
