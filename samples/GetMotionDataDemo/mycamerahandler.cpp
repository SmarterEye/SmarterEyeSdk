#include <iostream>
#include <string>

#include "mycamerahandler.h"
#include "satpext.h"
#include "frameid.h"
#include "motiondata.h"

MyCameraHandler::MyCameraHandler(const std::string &name):
    mName(name)
{}

void MyCameraHandler::handleRawFrame(const RawImageFrame *rawFrame)
{
    // get image frames here...
//    std::cout << "camera handler got frame time:" << rawFrame->time << std::endl;
//    processFrame(rawFrame->frameId, (char*)rawFrame + rawFrame->dataSize + sizeof(RawImageFrame),
//                 rawFrame->index);
}

void MyCameraHandler::handleMotionData(const MotionData *motionData)
{
    // get motiondata(imu) here...
    std::cout << "=======================" << std::endl;
    std::cout << "accel x: " << motionData->accelX << " g" << std::endl;
    std::cout << "accel y: " << motionData->accelY << " g" << std::endl;
    std::cout << "accel z: " << motionData->accelZ << " g" << std::endl;
    std::cout << "gyro x: " << motionData->gyroX << " deg/s" << std::endl;
    std::cout << "gyro y: " << motionData->gyroY << " deg/s" << std::endl;
    std::cout << "gyro z: " << motionData->gyroZ << " deg/s" << std::endl;
    std::cout << "time stamp: " << motionData->timestamp << " ms" << std::endl;
    std::cout << "temperature: " << motionData->temperature << " oC" << std::endl;
    std::cout << "=======================" << std::endl << std::endl;
}

void MyCameraHandler::processFrame(int frameId, const char *extended, size_t size)
{
    // custon function to deal with frames
}
