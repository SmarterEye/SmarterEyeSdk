//
// Created by xtp on 2020/3/9.
//

#ifndef ZKHY_STEREO_D_STEREO_PUBLISHER_D_H
#define ZKHY_STEREO_D_STEREO_PUBLISHER_D_H

#include <ros/ros.h>
#include "std_msgs/String.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.h"

class StereoCamera;
class FrameMonitor;

class StereoPublisher
{
public:
    StereoPublisher();
    ~StereoPublisher();

    void foo();

private:
    ros::NodeHandle node_handler_;
    image_transport::ImageTransport it_;
    image_transport::Publisher left_gray_pub_;
    image_transport::Publisher left_color_pub_;
    image_transport::Publisher right_gray_pub_;
    image_transport::Publisher right_color_pub_;
    image_transport::Publisher disparity_pub;

    std::unique_ptr<StereoCamera> stereo_camera_;
    std::unique_ptr<FrameMonitor> frame_monitor_;
};


#endif //ZKHY_STEREO_D_STEREO_PUBLISHER_D_H
