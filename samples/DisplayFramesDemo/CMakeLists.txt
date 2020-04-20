cmake_minimum_required(VERSION 3.1)

project(DisplayFramesDemo)

include(../common.cmake)

# find opencv
include(../find_opencv.cmake)
if(NOT OPENCV_FOUND)
    message(FATAL_ERROR "OpenCV Not Found!!!")
endif()

include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_DIR})

if (WIN32)
    link_libraries(${OpenCV_LIBS})
else()
    link_libraries(${OpenCV_LIBS} pthread)
endif()

add_executable(DisplayFramesDemo
    main.cpp framemonitor.cpp
)

set_target_properties(DisplayFramesDemo
    PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY ${BIN_DIR}
    LIBRARY_OUTPUT_DIRECTORY ${BIN_DIR}
    RUNTIME_OUTPUT_DIRECTORY ${BIN_DIR}
)
