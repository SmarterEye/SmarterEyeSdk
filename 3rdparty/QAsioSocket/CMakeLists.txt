cmake_minimum_required(VERSION 3.0)

project(QAsioSocket LANGUAGES C CXX)

add_definitions(-DQASIOSOCKET_LIBRARY)
add_definitions(-DASIO_STANDALONE)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

if(CMAKE_VERSION VERSION_LESS "3.7.0")
    set(CMAKE_INCLUDE_CURRENT_DIR ON)
endif()

find_package(Qt5 COMPONENTS Core REQUIRED)

include_directories(${Qt5Core_INCLUDE_DIRS})

include_directories(include)
include_directories(src)
include_directories(src/include_asio)

set (SRC_LIST
        src/qasiotcpsocket.cpp
        src/qasiotcpserver.cpp
        src/qsocketconnection.cpp
        )
add_library(QAsioSocket STATIC ${SRC_LIST})
target_link_libraries(QAsioSocket Qt5::Core)
