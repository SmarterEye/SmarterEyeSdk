#ifndef DEVICESTATUS_H
#define DEVICESTATUS_H

#include <QObject>
#include <QMutex>

#include "sedevicestate.h"

class DeviceReporterReplica;

class DeviceStatus : public QObject
{
    Q_OBJECT
public:
    static DeviceStatus *instance();
    void handleDeviceStateChanged(int state, QVector<int> errorCodeList);
    const SEDeviceState &getDeviceState();
    void acquireReplica(DeviceReporterReplica *replica);

signals:

private:
    explicit DeviceStatus(QObject *parent = nullptr);

    SEDeviceState *mSeDeviceState = nullptr;
    DeviceReporterReplica *mReplica = nullptr;
    QMutex mAccessMutex;
};

#endif // DEVICESTATUS_H
