add_definitions(-std=c++11)

message("Current folder is: ${CMAKE_CURRENT_SOURCE_DIR}")

if(NOT CMAKE_DEBUG_POSTFIX)
    set(CMAKE_DEBUG_POSTFIX d)
endif()

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
    set(SDK_BIN_DIR ${SDK_RELEASE_ROOT}/_output/bin)
    set(SDK_LIB_DIR ${SDK_RELEASE_ROOT}/_output/lib)
    set(SDK_LIBS smarter_eye_sdk)

    include_directories(${SDK_INCLUDE_DIR})
    link_directories(${SDK_BIN_DIR} ${SDK_LIB_DIR} ${SDK_LIB_DIR}/3rdparty)
    link_libraries(${SDK_LIBS})

endif()

set(BIN_DIR "${CMAKE_CURRENT_LIST_DIR}/../_output/bin")

if (WIN32)
  # set stuff for windows
    if(NOT CMAKE_SIZEOF_VOID_P EQUAL 8)
        message(FATAL_ERROR "Only x64 is supported!\n please use '-G 'Visual Studio 15 2017 Win64'' as argument")
    endif()
else()
  # set stuff for other systems
endif()
