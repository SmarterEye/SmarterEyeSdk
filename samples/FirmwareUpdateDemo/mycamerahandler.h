#ifndef MYIMAGEHANDLER_H
#define MYIMAGEHANDLER_H
#include "camerahandler.h"
#include <string>

class MyCameraHandler: public CameraHandler
{
public:
    MyCameraHandler(std::string name);
    void handleRawFrame(const RawImageFrame *rawFrame) {}
    void handleUpdateFinished(Result result);

private:
    std::string mName;
    Result mUpgradeResult;
};

#endif // MYIMAGEHANDLER_H
