cmake_minimum_required(VERSION 3.0)

project(PointCloudDemo)

include(../common.cmake)
include(../find_pcl.cmake)

if(NOT PCL_FOUND)
    message(FATAL_ERROR "PCL Not Found!!!")
endif()

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
link_libraries(${PCL_LIBRARIES})
add_definitions(${PCL_DEFINITIONS})

add_executable(PointCloudDemo
    main.cpp mycamerahandler.cpp pcviewer.cpp
    mycamerahandler.h pcviewer.h
)

set_target_properties(PointCloudDemo
    PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY "${BIN_DIR}"
    LIBRARY_OUTPUT_DIRECTORY "${BIN_DIR}"
    RUNTIME_OUTPUT_DIRECTORY "${BIN_DIR}"
)
