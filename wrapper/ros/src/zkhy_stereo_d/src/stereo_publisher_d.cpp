//
// Created by xtp on 2020/3/9.
//

#include "stereo_publisher_d.h"

#include "framemonitor.h"
#include "smarter_eye_sdk/stereocamera.h"
#include "smarter_eye_sdk/frameid.h"
#include "smarter_eye_sdk/calibrationparams.h"
#include "smarter_eye_sdk/rotationmatrix.h"

#include "cv_bridge/cv_bridge.h"
#include "std_msgs/String.h"


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

    camera_params_server_ = node_handler_.advertiseService("/zkhy_stereo/get_camera_params", &StereoPublisher::onCameraParamsRequest, this);
    rotation_matrix_server_ = node_handler_.advertiseService("/zkhy_stereo/get_rotation_matrix", &StereoPublisher::onRotationMatrixRequest, this);

    stereo_camera_->requestFrame(frame_monitor_.get(), FrameId::CalibLeftCamera | FrameId::CalibRightCamera | FrameId::Disparity);
}

StereoPublisher::~StereoPublisher()
{
    stereo_camera_->disconnectFromServer();
}

void StereoPublisher::publishFrames()
{
    frame_monitor_->waitForFrames();
    auto left_mat = frame_monitor_->getFrameMat(FrameId::CalibLeftCamera);
    auto right_mat = frame_monitor_->getFrameMat(FrameId::CalibRightCamera);
    auto disparity_mat = frame_monitor_->getFrameMat(FrameId::Disparity);

    bool is_left_color = (left_mat.type() == CV_8UC3);
    bool is_right_color = (right_mat.type() == CV_8UC3);
    std::string left_encoding = is_left_color ? "rgb8" : "mono8";
    std::string right_encoding = is_right_color ? "rgb8" : "mono8";

    sensor_msgs::ImagePtr left_msg, right_msg, disparity_msg;
    left_msg = cv_bridge::CvImage(std_msgs::Header(), left_encoding, left_mat).toImageMsg();
    right_msg = cv_bridge::CvImage(std_msgs::Header(), right_encoding, right_mat).toImageMsg();
    disparity_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", disparity_mat).toImageMsg();

    if (is_left_color) {
        left_color_pub_.publish(left_msg);
    } else {
        left_gray_pub_.publish(left_msg);
    }

    if (is_right_color) {
        right_color_pub_.publish(right_msg);
    } else {
        right_gray_pub_.publish(right_msg);
    }

    disparity_pub_.publish(disparity_msg);
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

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "stereo_publisher_d");

    StereoPublisher stereo_publisher;

    while (ros::ok()) {
        stereo_publisher.publishFrames();

        ros::spinOnce();
    }

    return 0;
}