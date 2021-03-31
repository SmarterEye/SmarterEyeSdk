if (WIN32)
    # set stuff for windows
#    SET(Qt5_PATH /path/to/your/qt/lib)
else()
    # set stuff for other systems
     SET(Qt5_PATH /opt/build-env/liugong/qt5.12.2/gcc-linaro-7.4.1-2019.02-x86_64_aarch64-linux-gnu)
endif()

SET(CMAKE_PREFIX_PATH ${Qt5_PATH})

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

if(CMAKE_VERSION VERSION_LESS "3.7.0")
    set(CMAKE_INCLUDE_CURRENT_DIR ON)
endif()

find_package(Qt5 COMPONENTS Core Network RemoteObjects REQUIRED)

include_directories(${Qt5Core_INCLUDE_DIRS} ${Qt5Network_INCLUDE_DIRS})
