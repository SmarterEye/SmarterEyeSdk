#ifndef MYCAMERAHANDLER_H
#define MYCAMERAHANDLER_H

#include "framehandler.h"
#include "motiondatahandler.h"

struct Result
{
    bool successed;
    int warning;
};

class CameraHandler : public FrameHandler, public MotionDataHandler
{
public:
    virtual void handleCameraDataFile(const char *path){}

    virtual void handleStereoCameraParam(float focus, float baseline, float pixelSize, int opticalCenterX, int opticalCenterY){}

    virtual void handleUpdateFinished(Result result){}

    virtual void handleMotionData(const MotionData *motionData) override {}
};

#endif // MYCAMERAHANDLER_H
