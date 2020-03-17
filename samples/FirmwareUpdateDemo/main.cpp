#include <chrono>
#include <thread>
#include <cstdio>
#include <fstream>

#include "stereocamera.h"
#include "protocol.h"
#include "frameid.h"
#include "mycamerahandler.h"

int main(int argc, char *argv[])
{
    if (argc < 2 || argv[1] == nullptr) {
        printf("please input the firmware package path. eg: ./FirmwareUpdateDemo SE_FirmWare_sm_2.0.1.tar.gz\n");
        return -1;
    }

    const char *packagePath = argv[1];

    std::fstream file;
    file.open(packagePath, std::ios::in);

    if (!file) {
        printf("%s, this firmware package does not exist!!!\n", packagePath);
        return -2;
    }

    StereoCamera *camera = StereoCamera::connect("192.168.1.251");
    MyCameraHandler *cameraHandlerA = new MyCameraHandler("camera A");

    camera->requestFrame(cameraHandlerA, FrameId::LeftCamera);

    while (!camera->isConnected()) {
        printf("connecting...\n");
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    printf("uploading %s\n", packagePath);
    camera->updateFirmware(packagePath);

    int c = 0;
    while (c!= 'x')
    {
        c = getchar();
    }
}
