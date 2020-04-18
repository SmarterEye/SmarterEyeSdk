#ifndef MOTIONDATARECEIVER_H
#define MOTIONDATARECEIVER_H

#include "rep_MotionData_replica.h"

class MotionDataHandler;

class MotionDataReceiver : public QObject
{
    Q_OBJECT

public:
    MotionDataReceiver();

    void init(const QString &ipAddr);

    void requestMotionData(MotionDataHandler *handler);

protected slots:
    void onMotionDataArrived(QByteArray motionData);

private:
    QRemoteObjectNode *mNode;
    MotionDataReplica *mReplica;

    QList<MotionDataHandler*> mHandlers;
};

#endif // MOTIONDATARECEIVER_H
