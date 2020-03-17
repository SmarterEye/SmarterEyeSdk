cmake_minimum_required(VERSION 3.0)

project(smarter-eye-sdk LANGUAGES C CXX)
set(SDK_VERSION 0.2.25)

include(cmake/common.cmake)

# config

set(SDK_NAME "${PROJECT_NAME}")
set(SDK_ROOT "${PROJECT_SOURCE_DIR}")

# packages

LIST(APPEND CMAKE_MODULE_PATH ${SDK_ROOT}/cmake)

include(cmake/find_opencv.cmake)
include(cmake/find_qt.cmake)

add_subdirectory(${SDK_ROOT}/3rdparty/QAsioSocket)

include_directories(${SDK_ROOT}/include)
include_directories(${SDK_ROOT}/3rdparty/QAsioSocket/include)

# src

set(SDK_MODULES
    src/MessageBus
    src/RealtimeDatabase
    src/RtdbService
    src/Satp
    src/SatpExt
    src/StereoCamera
    )

add_definitions(
    -DMESSAGEBUS_LIBRARY
    -DREALTIMEDATABASE_LIBRARY
    -DRTDBSERVICE_LIBRARY
    -DSATP_LIBRARY
    -DSATPEXT_LIBRARY
    -DSTEREOCAMERA_LIBRARY
    )

foreach(MODULE_DIR ${SDK_MODULES})
    include_directories(${MODULE_DIR})
    aux_source_directory(${MODULE_DIR} SDK_SRCS)
endforeach()

add_library(smarter_eye STATIC ${SDK_SRCS})
add_dependencies(smarter_eye QAsioSocket)
target_link_libraries(smarter_eye Qt5::Core Qt5::Network)
set_target_properties(smarter_eye PROPERTIES
        VERSION ${SDK_VERSION}
        )
