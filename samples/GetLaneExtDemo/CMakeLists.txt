cmake_minimum_required(VERSION 3.0)

project(GetLaneExtDemo)

include(../common.cmake)

add_executable(GetLaneExtDemo
    main.cpp mycamerahandler.cpp
)

set_target_properties(GetLaneExtDemo
    PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY "${BIN_DIR}"
    LIBRARY_OUTPUT_DIRECTORY "${BIN_DIR}"
    RUNTIME_OUTPUT_DIRECTORY "${BIN_DIR}"
)
