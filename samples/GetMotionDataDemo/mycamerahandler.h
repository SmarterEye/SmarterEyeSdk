#ifndef MYIMAGEHANDLER_H
#define MYIMAGEHANDLER_H
#include "camerahandler.h"
#include "motiondata.h"
#include <string>
#include <list>
#include <mutex>

class MyCameraHandler: public CameraHandler
{
public:
    MyCameraHandler(const std::string &name);

    void handleRawFrame(const RawImageFrame *rawFrame) override;

    void handleMotionData(const MotionData *motionData) override;

protected:
    void processFrame(int frameId, const char *extended, size_t size);

private:
    std::string mName;
    std::mutex mMutex;
};

#endif // MYIMAGEHANDLER_H
