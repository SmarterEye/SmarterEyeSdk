cmake_minimum_required(VERSION 3.0)

project(FirmwareUpdateDemo)

include(../common.cmake)

add_executable(FirmwareUpdateDemo
    main.cpp mycamerahandler.cpp
)

set_target_properties(FirmwareUpdateDemo
    PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY "${BIN_DIR}"
    LIBRARY_OUTPUT_DIRECTORY "${BIN_DIR}"
    RUNTIME_OUTPUT_DIRECTORY "${BIN_DIR}"
)
