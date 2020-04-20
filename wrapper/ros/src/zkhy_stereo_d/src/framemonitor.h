#ifndef FRAMEMONITOR_H
#define FRAMEMONITOR_H

#include <thread>
#include <mutex>
#include <memory>
#include <functional>
#include <condition_variable>
#include <opencv2/core.hpp>
#include <utility>

#include "smarter_eye_sdk/camerahandler.h"
#include "smarter_eye_sdk/motiondata.h"

class FrameMonitor : public CameraHandler
{
public:
    FrameMonitor();
    virtual ~FrameMonitor() {}

    // image handler func in recv thread of SATP Protocol(based on tcp)
    void handleRawFrame(const RawImageFrame *rawFrame);

    void handleMotionData(const MotionData *motionData) override;

    // custom function for processing recieved frame data in handleRawFrame()
    void processFrame(const RawImageFrame *rawFrame);

    void waitForFrames();

    cv::Mat getFrameMat(int frameId);

    void setFrameCallback(std::function<void(int frameId, const cv::Mat &imgMat)> cb) {mFrameCallback = std::move(cb);}

    void setMotionDataCallback(std::function<void(const MotionData *motionData)> cb) {mMotionDataCallback = std::move(cb);}

protected:
    void loadFrameData2Mat(const RawImageFrame *frameData, cv::Mat &dstMat);

private:
    std::mutex mMutex;
    std::condition_variable mFrameReadyCond;
    bool mFrameReadyFlag;

    cv::Mat mLeftMat;
    cv::Mat mRightMat;
    cv::Mat mDisparityMat;

    std::function<void(int frameId, const cv::Mat &imgMat)> mFrameCallback;
    std::function<void(const MotionData *motionData)> mMotionDataCallback;

    std::unique_ptr<float[]> mDisparityFloatData;
    std::unique_ptr<float []> mDisparityDistanceZ;
    std::unique_ptr<unsigned char[]> mRgbBuffer;
};

#endif // FRAMEMONITOR_H
