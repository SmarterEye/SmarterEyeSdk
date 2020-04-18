#ifndef FIRMWAREUPDATER_H
#define FIRMWAREUPDATER_H

#include <QObject>
#include <QUdpSocket>
#include <QThread>
#include <QMap>
#include "stereocamera_global.h"
#include "blockhandler.h"
#include "filesenderhandler.h"

namespace SATP {
    class Protocol;
    class FileSender;
}
class DeviceReporterReplica;

class STEREOCAMERASHARED_EXPORT FirmwareUpdater : public QObject,
        public SATP::BlockHandler,
        public SATP::FileSenderHandler
{
    Q_OBJECT
public:
    enum UpdateStatus{
        Idle = 0,
        Uploading,
        Upgrading
    };

    explicit FirmwareUpdater(SATP::Protocol *protocol, QObject *parent = nullptr);
    virtual ~FirmwareUpdater();

    //override BlockHandler
    bool handleReceiveBlock(quint32 dataType, const char *block, int size);
    void handleReady();
    void handleReset();

    //override FileSenderHandler
    void handleSendFileFinished(const QString &fileName);
    void handleUpgradeProgress(int progress);
    int getUpdateStatus();
    int getUpdateWarning(){return mUpdateWarning;}
    void initSocket();
    void acquireReplica(DeviceReporterReplica *replica);


signals:
    void updateEvent(const QString &event);

public slots:
    int update(const QString &path);
    double getUpdateProgress();
    bool isDeviceHighTemperature();

protected:
    void handleUpdateResp(const char *block, int size);
    void triggerEvent(const QString &event);
    void handleDeviceStateChanged(int state, QVector<int> errorCodeList);
    void timerEvent(QTimerEvent *event);
    void startProgressCheckTimer();
    void decodeUDPdatagram(QString datagram);
    bool isReplicaValid() const;

protected slots:
    void processPendingDatagram();
    void onErrorOccur(QAbstractSocket::SocketError sockError);

private:
    SATP::Protocol *mProtocol;
    SATP::FileSender *mFileSender;
    DeviceReporterReplica *mReplica = nullptr;
    UpdateStatus mUpdateStatus;
    QByteArray mFirmwareMd5;
    int mUpgradeProgress;
    bool mDeviceFailure;
    bool mHighTemperature;
    int mErrorCode;
    int mUpdateWarning;
    int mProgressCheckTimerId;
    QUdpSocket *mUdpSocket;
    QString mProductModel;
};

#endif // FIRMWAREUPDATER_H
