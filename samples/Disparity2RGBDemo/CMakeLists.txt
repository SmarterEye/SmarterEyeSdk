cmake_minimum_required(VERSION 3.0)

project(Disparity2RGBDemo)

include(../common.cmake)

add_executable(Disparity2RGBDemo
    main.cpp mycamerahandler.cpp
)

set_target_properties(Disparity2RGBDemo
    PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY "${BIN_DIR}"
    LIBRARY_OUTPUT_DIRECTORY "${BIN_DIR}"
    RUNTIME_OUTPUT_DIRECTORY "${BIN_DIR}"
)
