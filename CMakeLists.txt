cmake_minimum_required(VERSION 3.0)

project(smarter_eye_sdk LANGUAGES C CXX)
set(SDK_VERSION 0.2.25)
add_definitions(-std=c++11)

include(cmake/common.cmake)

# config

set(SDK_NAME "${PROJECT_NAME}")
set(SDK_ROOT "${PROJECT_SOURCE_DIR}")

# packages

LIST(APPEND CMAKE_MODULE_PATH ${SDK_ROOT}/cmake)

include(cmake/find_qt.cmake)

include_directories(${SDK_ROOT}/include/${SDK_NAME})
include_directories(${SDK_ROOT}/3rdparty/QAsioSocket/include)

# src

set(SDK_MODULES
    src/MessageBus
    src/RealtimeDatabase
    src/RtdbService
    src/Satp
    src/SatpExt
    src/StereoCamera
    src/ImageUtils
    )

add_definitions(
    -DMESSAGEBUS_LIBRARY
    -DREALTIMEDATABASE_LIBRARY
    -DRTDBSERVICE_LIBRARY
    -DSATP_LIBRARY
    -DSATPEXT_LIBRARY
    -DSTEREOCAMERA_LIBRARY
    -DIMAGEUTILS_LIBRARY
    )


set(SDK_INCS)
foreach(MODULE_DIR ${SDK_MODULES})
    include_directories(${MODULE_DIR})
    aux_source_directory(${MODULE_DIR} SDK_SRCS)
    file(GLOB CUR_INCS "${MODULE_DIR}/*.h")
    list(APPEND SDK_INCS ${CUR_INCS})
endforeach()

# output

set(OUTPUT_DIR "${SDK_ROOT}/_output")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${OUTPUT_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${OUTPUT_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${OUTPUT_DIR}/bin)

# submodule
add_subdirectory(${SDK_ROOT}/3rdparty/QAsioSocket)
set_target_properties(QAsioSocket
    PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY ${OUTPUT_DIR}/lib/3rdparty
    LIBRARY_OUTPUT_DIRECTORY ${OUTPUT_DIR}/lib/3rdparty
    RUNTIME_OUTPUT_DIRECTORY ${OUTPUT_DIR}/bin/3rdparty
    )

# rpath
set(SDK_CMAKE_RPATH
    "${CMAKE_INSTALL_PREFIX}/lib/3rdparty"
    "${Qt5_PATH}/lib"
    )

# build shared
qt5_wrap_cpp(MOC_SRC ${SDK_SRCS} ${SDK_INCS})
add_library(${SDK_NAME} SHARED ${SDK_SRCS} ${SDK_INCS} ${MOC_SRC})
add_dependencies(${SDK_NAME} QAsioSocket)
target_link_libraries(${SDK_NAME}
    PRIVATE $<BUILD_INTERFACE:QAsioSocket>
    Qt5::Core Qt5::Network
    )
set_target_properties(${SDK_NAME} PROPERTIES
    VERSION ${SDK_VERSION}
    INSTALL_RPATH "${SDK_CMAKE_RPATH}"
    )

# install
set(SDK_CMAKE_INCLUDE_DIR "${CMAKE_INSTALL_PREFIX}/include/${SDK_NAME}")
set(SDK_CMAKE_BINDIR "${CMAKE_INSTALL_PREFIX}/bin")
set(SDK_CMAKE_LIBDIR "${CMAKE_INSTALL_PREFIX}/lib")
set(SDK_CMAKE_INSTALLDIR "${SDK_CMAKE_LIBDIR}/cmake/${SDK_NAME}")

install(TARGETS ${SDK_NAME}
    EXPORT ${SDK_NAME}-targets
    ARCHIVE DESTINATION ${SDK_CMAKE_LIBDIR}
    LIBRARY DESTINATION ${SDK_CMAKE_LIBDIR}
    RUNTIME DESTINATION ${SDK_CMAKE_BINDIR}
    )

if(OS_WIN)
    set(__3rdparty_src_dir ${OUTPUT_DIR}/bin/3rdparty)
    set(__3rdparty_dst_dir ${SDK_CMAKE_BINDIR})
else()
    set(__3rdparty_src_dir ${OUTPUT_DIR}/lib/3rdparty)
    set(__3rdparty_dst_dir ${SDK_CMAKE_LIBDIR})
endif()

install(DIRECTORY "${__3rdparty_src_dir}"
    DESTINATION "${__3rdparty_dst_dir}"
    FILES_MATCHING PATTERN "*.so*" PATTERN "*.dll" PATTERN "*.dylib"
    )

install(DIRECTORY
    "${SDK_ROOT}/include"
    DESTINATION ${CMAKE_INSTALL_PREFIX}
    FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
    )

include(CMakePackageConfigHelpers)

write_basic_package_version_file(
    ${SDK_NAME}-version.cmake
    VERSION ${SDK_VERSION}
    COMPATIBILITY SameMajorVersion
    )

export(
    EXPORT ${SDK_NAME}-targets
    FILE ${CMAKE_CURRENT_BINARY_DIR}/${SDK_NAME}-targets.cmake
    )

configure_package_config_file(${SDK_NAME}-config.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/${SDK_NAME}-config.cmake
  INSTALL_DESTINATION ${SDK_CMAKE_INSTALLDIR}
)

install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/${SDK_NAME}-config.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/${SDK_NAME}-version.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/${SDK_NAME}-targets.cmake
  DESTINATION ${SDK_CMAKE_INSTALLDIR}
)
