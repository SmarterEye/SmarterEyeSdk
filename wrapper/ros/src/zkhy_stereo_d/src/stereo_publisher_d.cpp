//
// Created by xtp on 2020/3/9.
//

#include "stereo_publisher_d.h"

#include "smarter_eye_sdk/stereocamera.h"
#include "smarter_eye_sdk/frameid.h"
#include "framemonitor.h"


StereoPublisher::StereoPublisher()
    : it_(node_handler_),
      stereo_camera_(StereoCamera::connect("192.168.1.251")),
      frame_monitor_(new FrameMonitor)
{
    left_gray_pub_ = it_.advertise("zkhy_stereo/left/gray", 1);

    stereo_camera_->requestFrame(frame_monitor_.get(), FrameId::CalibLeftCamera);
}

StereoPublisher::~StereoPublisher()
{

}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "stereo_publisher_d");

    StereoPublisher stereo_publisher;
    ros::spin();

    return 0;
}