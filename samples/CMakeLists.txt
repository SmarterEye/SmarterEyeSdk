cmake_minimum_required (VERSION 3.0)

project(Samples)

# Add sub directories
add_subdirectory(StereoCameraDemo)
add_subdirectory(Disparity2Real3dDemo)
add_subdirectory(Disparity2RGBDemo)
add_subdirectory(FirmwareUpdateDemo)
add_subdirectory(GetCameraParamDemo)
add_subdirectory(GetCompoundDemo)
add_subdirectory(GetLaneExtDemo)
add_subdirectory(GetMotionDataDemo)

include(find_opencv.cmake)
if (${OPENCV_FOUND})
    add_subdirectory(DisplayFramesDemo)
endif()

include(find_pcl.cmake)
if (${PCL_FOUND} AND ${PCL_VERSION} VERSION_GREATER_EQUAL 1.9)
    add_subdirectory(PointCloudDemo)
endif()
