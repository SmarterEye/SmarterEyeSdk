#ifndef STEREOCAMERAIMPL_H
#define STEREOCAMERAIMPL_H

#include "stereocamera_global.h"

#include <QObject>

#include "blockhandler.h"
#include "filereceiverhandler.h"
#include "filesenderhandler.h"
#include "taskiddef.h"
#include "devicestatus.h"

class QCoreApplication;
class Protocol;
class CameraHandler;
class FrameHandler;
class FrameReceiver;
class FirmwareUpdater;
class MotionDataHandler;
class MotionDataReceiver;

struct StereoCalibrationParameters;
struct MonoCalibrationParameters;
struct RotationMatrix;

namespace SATP {
class Protocol;
class FileReceiver;
class Connector;
class FileSender;
}

class QRemoteObjectNode;
class DeviceControllerReplica;
class DeviceReporterReplica;

class STEREOCAMERASHARED_EXPORT StereoCameraImpl : public QObject,
        public SATP::BlockHandler,
        public SATP::FileReceiverHandler,
        public SATP::FileSenderHandler
{
    Q_OBJECT
    Q_PROPERTY(int camImageWidth MEMBER mImageWidth NOTIFY camImageWidthChanged)
    Q_PROPERTY(int camImageHeight MEMBER mImageHeight NOTIFY camImageHeightChanged)
    Q_PROPERTY(int camLensFocus MEMBER mLensFocus NOTIFY camLensFocusChanged)
    Q_PROPERTY(float camSensorPixelSize MEMBER mSensorPixelSize NOTIFY camSensorPixelSizeChanged)

public:
    explicit StereoCameraImpl(QObject *parent = nullptr);

    virtual ~StereoCameraImpl() override;

    SATP::Protocol *getProtocol();

    inline SATP::Connector *getConnector(){return mConnector;}

    void controlTasks();

    void requestFrame(FrameHandler *handler, uint32_t frameIds);

    void requestMotionData(MotionDataHandler *handler);

    void reboot(bool halt);

    void enableTasks(uint32_t taskIds);

    bool requestStereoCameraParameters(StereoCalibrationParameters &params);

    bool requestMonoLeftCameraParameters(MonoCalibrationParameters &params);

    bool requestMonoRightCameraParameters(MonoCalibrationParameters &params);

    bool requestRotationMatrix(RotationMatrix &rotationMatrix);

    void enableMaxSendFrameInterval();

    bool setFrameRate(float rate);

    bool getFrameRate(float &rate);

    uint32_t getActiveTaskIds() {return mTaskIds;}

    bool getAmbientLight(int &lightness);

    bool getSmudgeStatus(int &status);

    void setImuParameter(const QString &name, int value);

    int getImuParameter(const QString &name);

    void enableMotionData(bool enable);

    const SEDeviceState &getDeviceState();

    //override BlockHandler
    bool handleReceiveBlock(quint32 dataType, const char *block, int size) override;

    void handleReady() override;

    void handleReset() override;

    //override FileReceiverHandler
    void handleReceiveFile(const QString &fileName) override;

    void handleSendFileFinished(const QString &fileName) override;

    inline FrameReceiver *getFrameReceiver(){return mFrameReceiver;}

    inline SATP::FileReceiver *getFileReceiver(){return mFileReceiver;}

    inline FirmwareUpdater *getFirmwareUpdater(){return mFirmwareUpdater;}

signals:
    void cameraConnected();
    void cameraDisconnected();
    void updateFinished(bool successed);
    void gotAvailableFrameIds(quint16 ids);
    void udpPackageReceived();
    void camImageWidthChanged(int width);
    void camImageHeightChanged(int height);
    void camLensFocusChanged(int lens);
    void camSensorPixelSizeChanged(float size);
    void fileSendFinished(const QString &filePath);

public slots:
    void connectTo(QString addr);
    void reconnect();
    void disconnectServer();
    bool isConnected();
    QString getAddress();
    int updateFirmware(QString path);
    double getUpdateProgress();
    int getUpdateStatus();
    void setReceiveDir(const QString &dir);
    int getUpdateWarning();
    bool isDeviceHighTemperature();
    bool sendFileToDevice(const QString &filePath);

protected slots:
    void onFirmwareUpdateEvent(const QString &event);
    void initRemoteNode();

protected:
    void init();
    bool isReplicaValid() const;

private:
    SATP::Connector *mConnector;
    FrameReceiver *mFrameReceiver;
    SATP::FileReceiver *mFileReceiver;
    SATP::FileSender *mFileSender;
    FirmwareUpdater *mFirmwareUpdater;

    MotionDataReceiver *mMotionDataReceiver;

    QRemoteObjectNode *mControllerNode;
    QRemoteObjectNode *mReporterNode;
    DeviceControllerReplica *mControllerReplica = nullptr;
    DeviceReporterReplica *mReporterReplica = nullptr;

    uint32_t mTaskIds;

    int mImageWidth;
    int mImageHeight;
    int mLensFocus;
    float mSensorPixelSize;
    bool mIsImuDataEnable;
};

#endif // STEREOCAMERAIMPL_H
