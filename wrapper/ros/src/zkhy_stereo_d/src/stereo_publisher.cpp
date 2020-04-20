//
// Created by xtp on 2020/3/9.
//

#include "stereo_publisher.h"

#include "framemonitor.h"
#include "smarter_eye_sdk/stereocamera.h"
#include "smarter_eye_sdk/frameid.h"
#include "smarter_eye_sdk/calibrationparams.h"
#include "smarter_eye_sdk/rotationmatrix.h"

#include "cv_bridge/cv_bridge.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Imu.h"

#include <cmath>


StereoPublisher::StereoPublisher()
    : it_(node_handler_),
      stereo_camera_(StereoCamera::connect("192.168.1.251")),
      frame_monitor_(new FrameMonitor)
{
    left_gray_pub_ = it_.advertise("/zkhy_stereo/left/gray", 1);
    left_color_pub_ = it_.advertise("/zkhy_stereo/left/color", 1);

    right_gray_pub_ = it_.advertise("/zkhy_stereo/right/gray", 1);
    right_color_pub_ = it_.advertise("/zkhy_stereo/right/color", 1);

    disparity_pub_ = it_.advertise("/zkhy_stereo/disparity", 1);

    imu_pub_ = node_handler_.advertise<sensor_msgs::Imu>("/zkhy_stereo/imu", 20);

    camera_params_server_ = node_handler_.advertiseService("/zkhy_stereo/get_camera_params", &StereoPublisher::onCameraParamsRequest, this);
    rotation_matrix_server_ = node_handler_.advertiseService("/zkhy_stereo/get_rotation_matrix", &StereoPublisher::onRotationMatrixRequest, this);

    stereo_camera_->requestFrame(frame_monitor_.get(), FrameId::CalibLeftCamera | FrameId::CalibRightCamera | FrameId::Disparity);
    stereo_camera_->requestMotionData(frame_monitor_.get());
    stereo_camera_->enableMotionData(true);

    frame_monitor_->setFrameCallback(std::bind(&StereoPublisher::publishFrameCallback, this, std::placeholders::_1, std::placeholders::_2));
    frame_monitor_->setMotionDataCallback(std::bind(&StereoPublisher::publishImuCallback, this, std::placeholders::_1));
}

StereoPublisher::~StereoPublisher()
{
    stereo_camera_->disconnectFromServer();
}

bool StereoPublisher::onCameraParamsRequest(zkhy_stereo_d::CameraParams::Request &req,
                                            zkhy_stereo_d::CameraParams::Response &resp)
{
    StereoCalibrationParameters params {};
    bool ok = stereo_camera_->requestStereoCameraParameters(params);
    if (ok) {
        resp.focus = params.focus;
        resp.cx = params.cx;
        resp.cy = params.cy;
        resp.RRoll = params.RRoll;
        resp.RPitch = params.RPitch;
        resp.RYaw = params.RYaw;
        resp.Tx = params.Tx;
        resp.Ty = params.Ty;
        resp.Tz = params.Tz;
    } else {
        std::cout << "Stereo camera params request failed from ADAS.";
    }

    return ok;
}

bool StereoPublisher::onRotationMatrixRequest(zkhy_stereo_d::RotationMatrix::Request &req,
                                              zkhy_stereo_d::RotationMatrix::Response &resp)
{
    RotationMatrix matrix {};
    bool ok = stereo_camera_->requestRotationMatrix(matrix);
    if (ok) {
        for (size_t i = 0; i < resp.real3DToImage.size(); i++) {
            resp.real3DToImage[i] = matrix.real3DToImage[i];
        }
        for (size_t j = 0; j < resp.imageToReal3D.size(); j++) {
            resp.imageToReal3D[j] = matrix.imageToReal3D[j];
        }
    } else {
        std::cout << "Rotation matrix request failed from ADAS.";
    }

    return ok;
}

void StereoPublisher::publishFrameCallback(int frameId, const cv::Mat &img_mat)
{
    bool is_color = (img_mat.type() == CV_8UC3);
    std::string encoding = is_color ? "rgb8" : "mono8";
    sensor_msgs::ImagePtr img_msg;

    if (frameId == FrameId::CalibLeftCamera || frameId == FrameId::CalibRightCamera) {
        img_msg = cv_bridge::CvImage(std_msgs::Header(), encoding, img_mat).toImageMsg();

        if (frameId == FrameId::CalibLeftCamera) {
            if (is_color) {
                left_color_pub_.publish(img_msg);
            } else {
                left_gray_pub_.publish(img_msg);
            }
        }

        if (frameId == FrameId::CalibRightCamera) {
            if (is_color) {
                right_color_pub_.publish(img_msg);
            } else {
                right_gray_pub_.publish(img_msg);
            }
        }
    }

    if (frameId == FrameId::Disparity) {
        img_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", img_mat).toImageMsg();
        disparity_pub_.publish(img_msg);
    }
}

void StereoPublisher::publishImuCallback(const MotionData *motionData)
{
    sensor_msgs::Imu imu_msg;
    imu_msg.header.stamp = ros::Time(motionData->timestamp);
    imu_msg.header.frame_id = "base_link";

    imu_msg.linear_acceleration.x = motionData->accelX;
    imu_msg.linear_acceleration.y = motionData->accelY;
    imu_msg.linear_acceleration.z = motionData->accelZ;

    imu_msg.angular_velocity.x = motionData->gyroX * M_PI / 180;
    imu_msg.angular_velocity.y = motionData->gyroY * M_PI / 180;
    imu_msg.angular_velocity.z = motionData->gyroZ * M_PI / 180;

    imu_pub_.publish(imu_msg);
}


int main(int argc, char* argv[])
{
    ros::init(argc, argv, "stereo_publisher_d");

    StereoPublisher stereo_publisher;

    ros::spin();

    return 0;
}