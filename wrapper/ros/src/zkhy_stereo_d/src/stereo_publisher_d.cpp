//
// Created by xtp on 2020/3/9.
//

#include "stereo_publisher_d.h"

#include "framemonitor.h"
#include "smarter_eye_sdk/stereocamera.h"
#include "smarter_eye_sdk/frameid.h"
#include "cv_bridge/cv_bridge.h"


StereoPublisher::StereoPublisher()
    : it_(node_handler_),
      stereo_camera_(StereoCamera::connect("192.168.3.19")),
      frame_monitor_(new FrameMonitor)
{
    left_gray_pub_ = it_.advertise("zkhy_stereo/left/gray", 1);

    stereo_camera_->requestFrame(frame_monitor_.get(), FrameId::CalibLeftCamera | FrameId::CalibRightCamera | FrameId::Disparity);
}

StereoPublisher::~StereoPublisher()
{

}

void StereoPublisher::foo()
{
    auto xx = frame_monitor_->getFrameMat(FrameId::CalibLeftCamera);
    sensor_msgs::ImagePtr msg;
    msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", xx).toImageMsg();
    left_gray_pub_.publish(msg);
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "stereo_publisher_d");

    StereoPublisher stereo_publisher;

    ros::Rate loop_rate(1);
    while (ros::ok()) {
        stereo_publisher.foo();

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}