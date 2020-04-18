#include "motiondatareceiver.h"

#include "motiondatahandler.h"

MotionDataReceiver::MotionDataReceiver() :
    mNode(nullptr),
    mReplica(nullptr)
{

}

void MotionDataReceiver::init(const QString &ipAddr)
{
    if (!mNode) {
        mNode = new QRemoteObjectNode;
        auto url = QUrl("tcp://" + ipAddr + ":" + QString::number(MotionDataReplica::NodePort::Default));
        mNode->connectToNode(url);
        mReplica = mNode->acquire<MotionDataReplica>();
        connect(mReplica, &MotionDataReplica::dataChanged, this, &MotionDataReceiver::onMotionDataArrived);
    }

    mReplica->waitForSource(3000);
}

void MotionDataReceiver::requestMotionData(MotionDataHandler *handler)
{
    if (!mHandlers.contains(handler)) {
        mHandlers << handler;
    }
}

void MotionDataReceiver::onMotionDataArrived(QByteArray motionData)
{
    struct MotionData *motionDataStruct = reinterpret_cast<MotionData*>(motionData.data());

    for (auto handler : mHandlers) {
        if (handler) {
            handler->handleMotionData(motionDataStruct);
        }
    }
}
