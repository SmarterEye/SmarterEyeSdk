cmake_minimum_required(VERSION 3.0)

project(Disparity2Real3dDemo)

include(../common.cmake)

add_executable(Disparity2Real3dDemo
    main.cpp mycamerahandler.cpp
)

set_target_properties(Disparity2Real3dDemo
    PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY "${BIN_DIR}"
    LIBRARY_OUTPUT_DIRECTORY "${BIN_DIR}"
    RUNTIME_OUTPUT_DIRECTORY "${BIN_DIR}"
)
