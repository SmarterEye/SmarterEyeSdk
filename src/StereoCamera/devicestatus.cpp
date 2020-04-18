#include "devicestatus.h"
#include "rep_DeviceReporter_replica.h"

DeviceStatus::DeviceStatus(QObject *parent) : QObject(parent)
{
    mSeDeviceState = new SEDeviceState();
}

DeviceStatus *DeviceStatus::instance()
{
    static DeviceStatus* inst = nullptr;
    if (!inst) {
        inst = new DeviceStatus;
        Q_ASSERT(inst);
    }
    return inst;
}

void DeviceStatus::handleDeviceStateChanged(int state, QVector<int> errorCodeList)
{
    QMutexLocker locker(&mAccessMutex);

    switch (state) {
    case SEDeviceState::NormalState:
    case SEDeviceState::AbnormalState:
    case SEDeviceState::HighTemperatureState:
        mSeDeviceState->setErrorCodeList(errorCodeList.toStdVector());
        mSeDeviceState->setDeviceState(static_cast<SEDeviceState::DeviceState>(state));
        break;
    }
}

const SEDeviceState &DeviceStatus::getDeviceState()
{
    QMutexLocker locker(&mAccessMutex);
    return *mSeDeviceState;
}

void DeviceStatus::acquireReplica(DeviceReporterReplica *replica)
{
    mReplica = replica;
    connect(mReplica, &DeviceReporterReplica::deviceStateChanged, this, &DeviceStatus::handleDeviceStateChanged);
}
