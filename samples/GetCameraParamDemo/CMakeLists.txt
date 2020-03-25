cmake_minimum_required(VERSION 3.0)

project(GetCameraParamDemo)

include(../common.cmake)

add_executable(GetCameraParamDemo main.cpp)

set_target_properties(GetCameraParamDemo
    PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY "${BIN_DIR}"
    LIBRARY_OUTPUT_DIRECTORY "${BIN_DIR}"
    RUNTIME_OUTPUT_DIRECTORY "${BIN_DIR}"
)
