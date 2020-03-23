#include <ros/ros.h>
#include "std_msgs/String.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.h"

class StereoListener
{
public:
    StereoListener();
    ~StereoListener();

    void grayCallback(const sensor_msgs::ImageConstPtr &msg);
    void colorCallback(const sensor_msgs::ImageConstPtr &msg);
    void disparityCallback(const sensor_msgs::ImageConstPtr &msg);

private:
    ros::NodeHandle node_handler_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber gray_sub_;
    image_transport::Subscriber color_sub_;
    image_transport::Subscriber disparity_sub_;
};

StereoListener::StereoListener()
        : it_(node_handler_)
{
    gray_sub_ = it_.subscribe("zkhy_stereo/left/gray", 1, &StereoListener::grayCallback, this);
//    color_sub_ = it_.subscribe("zkhy_stereo/left/color", 1, &StereoListener::colorCallback, this);
//    disparity_sub_ = it_.subscribe("zkhy_stereo/disparity/raw", 1, &StereoListener::disparityCallback, this);
}

StereoListener::~StereoListener()
{

}

void StereoListener::grayCallback(const sensor_msgs::ImageConstPtr &msg)
{
    cv_bridge::CvImageConstPtr image_ptr;
    try {
        image_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::TYPE_8UC1);
    } catch (cv_bridge::Exception &e) {
        return;
    }

    std::cout << image_ptr->image.cols << " : " << image_ptr->image.rows << std::endl;
}

void StereoListener::colorCallback(const sensor_msgs::ImageConstPtr &msg)
{
    cv_bridge::CvImageConstPtr image_ptr;
    try {
        image_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8);
    } catch (cv_bridge::Exception &e) {
        return;
    }

    std::cout << image_ptr->image.cols;
}

void StereoListener::disparityCallback(const sensor_msgs::ImageConstPtr &msg)
{
    cv_bridge::CvImageConstPtr image_ptr;
    try {
        image_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::TYPE_16UC1);
    } catch (cv_bridge::Exception &e) {
        return;
    }

    std::cout << image_ptr->image.cols;
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "stereo_listener_d");

    StereoListener stereo_listener;
    ros::spin();

    return 0;
}
