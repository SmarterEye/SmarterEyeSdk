#include <iostream>
#include <string>

#include "mycamerahandler.h"
#include "satpext.h"
#include "frameid.h"
#include "LdwDataInterface.h"
#include "roadwaypainter.h"
#include "frameext.h"

MyCameraHandler::MyCameraHandler(std::string name):
    mName(name)
{}

void MyCameraHandler::handleRawFrame(const RawImageFrame *rawFrame)
{
//    std::cout << mName
//              << ", got image, id: " << rawFrame->frameId
//              << " , time stamp: " << rawFrame->time
//              << std::endl;

    // put you image processing logic here.    eg:
    processFrame(rawFrame->frameId, (char*)rawFrame + sizeof(RawImageFrame),
                 (char*)rawFrame + rawFrame->dataSize + sizeof(RawImageFrame),
                 rawFrame->time, rawFrame->width, rawFrame->height);
}

void MyCameraHandler::processFrame(int frameId, char *image, char *extended, int64_t time, int width, int height)
{
    switch (frameId) {
    case FrameId::LaneExt:
    {
        //extended is LaneInfo of this frame,you can use it to paintRoadway or print.
        FrameDataExtHead *header = reinterpret_cast<FrameDataExtHead *>(extended);
        LdwDataPack *ldwDataPack = (LdwDataPack *)header->data;
        std::cout << "ldw degree is: " << ldwDataPack->roadway.left_Lane.left_Boundary.degree << std::endl;

        static unsigned char *rgbBuf = new unsigned char[width * height * 3];
        RoadwayPainter::imageGrayToRGB((unsigned char*)image, rgbBuf, width, height);
        mIsLaneDetected = RoadwayPainter::paintRoadway(header->data, rgbBuf, width, height);
    }
        break;
    default:
        break;
    }
}
