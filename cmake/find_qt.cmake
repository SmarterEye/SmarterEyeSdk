if (WIN32)
    # set stuff for windows
else()
    # set stuff for other systems
    # if you build custom opencv in linux, set following OpenCV_Dir
     SET(CMAKE_PREFIX_PATH /opt/Qt5.12.7/5.12.7/gcc_64)
endif()

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

if(CMAKE_VERSION VERSION_LESS "3.7.0")
    set(CMAKE_INCLUDE_CURRENT_DIR ON)
endif()

find_package(Qt5 COMPONENTS Core Network REQUIRED)

include_directories(${Qt5Core_INCLUDE_DIRS} ${Qt5Network_INCLUDE_DIRS})
