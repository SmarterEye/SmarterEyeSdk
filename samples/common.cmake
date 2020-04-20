add_definitions(-std=c++11)

message("Current folder is: ${CMAKE_CURRENT_SOURCE_DIR}")

if(WIN32)
    set(CMAKE_PREFIX_PATH ${CMAKE_CURRENT_LIST_DIR}/../install)
endif()

find_package(smarter_eye_sdk)

if(smarter_eye_sdk_FOUND)

    message(STATUS "use installed smarter_eye_sdk from ${smarter_eye_sdk_LIBRARY_DIRS}")
    message(STATUS ${smarter_eye_sdk_INCLUDE_DIRS})
    message(STATUS ${smarter_eye_sdk_LIBRARY_DIRS})
    message(STATUS ${smarter_eye_sdk_LIBRARIES})

    include_directories(${smarter_eye_sdk_INCLUDE_DIRS})
    link_directories(${smarter_eye_sdk_LIBRARY_DIRS})
    link_libraries(${smarter_eye_sdk_LIBRARIES})

else()

    message(STATUS "try to use released binary libs")

    set(SDK_RELEASE_ROOT "${CMAKE_CURRENT_LIST_DIR}/..")
    set(SDK_INCLUDE_DIR ${SDK_RELEASE_ROOT}/include/smarter_eye_sdk)
    set(SDK_BIN_DIR ${SDK_RELEASE_ROOT}/bin)
    set(SDK_LIB_DIR ${SDK_BIN_DIR})
    set(SDK_LIBS StereoCamera ImageUtils)

    include_directories(${SDK_INCLUDE_DIR})
    link_directories(${SDK_BIN_DIR})
    link_libraries(${SDK_LIBS})

endif()

set(BIN_DIR "${CMAKE_CURRENT_LIST_DIR}/../bin")

if (WIN32)
  # set stuff for windows
    if(NOT CMAKE_SIZEOF_VOID_P EQUAL 8)
        message(FATAL_ERROR "Only x64 is supported!\n please use '-G 'Visual Studio 15 2017 Win64'' as argument")
    endif()
else()
  # set stuff for other systems
endif()
