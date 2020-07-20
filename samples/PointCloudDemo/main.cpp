#include <iostream>
#include <string>
#include <chrono>
#include <thread>

#include "stereocamera.h"
#include "frameid.h"
#include "taskiddef.h"
#include "mycamerahandler.h"
#include "protocol.h"
#include "calibrationparams.h"
#include "pcviewer.h"

int main(int argc, char *argv[])
{
    StereoCamera *cameraA = StereoCamera::connect("192.168.1.251");
    MyCameraHandler *cameraHandlerA = new MyCameraHandler("camera A");

    while (!cameraA->getProtocol()->isConnected()) {
        std::cout << "connecting..." << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    std::this_thread::sleep_for(std::chrono::seconds(3));

    cameraA->enableTasks(TaskId::DisplayTask);
    cameraA->enableMaxSendFrameInterval();

    StereoCalibrationParameters params;
    cameraA->requestStereoCameraParameters(params);
    cameraHandlerA->setStereoCalibParams(params);
    cameraA->requestFrame(cameraHandlerA, FrameId::Disparity);

    std::shared_ptr<PCViewer> viewer(new PCViewer(true));

    // main loop for showing point cloud
    while (!viewer->wasStopped()) {
        auto cloud = cameraHandlerA->getCloud();
        viewer->update(cloud);
    }

    std::cout << "stopped" << std::endl;
    viewer->close();

    cameraA->disconnectFromServer();
    delete cameraA;
    delete cameraHandlerA;

    return 0;
}
