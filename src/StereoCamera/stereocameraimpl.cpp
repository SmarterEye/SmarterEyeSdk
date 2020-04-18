#include <QCoreApplication>
#include <QTimer>
#include <QDebug>
#include <QThreadPool>

#include "stereocameraimpl.h"
#include "protocol.h"
#include "connector.h"
#include "framereceiver.h"
#include "filereceiver.h"
#include "camerahandler.h"
#include "firmwareupdater.h"
#include "taskiddef.h"
#include "calibrationparams.h"
#include "rotationmatrix.h"
#include "filesender.h"
#include "motiondatahandler.h"
#include "motiondatareceiver.h"

#include "rep_DeviceController_replica.h"
#include "rep_DeviceReporter_replica.h"


#ifdef Q_OS_LINUX
static const QString kAdasDownloadPath(QString(getenv("HOME")) + "/ADAS_Files/");
#else
static const QString kAdasDownloadPath("D:/");
#endif

StereoCameraImpl::StereoCameraImpl(QObject *parent)
    : QObject(parent),
      mConnector(nullptr),
      mFrameReceiver(nullptr),
      mFileReceiver(nullptr),
      mFileSender(nullptr),
      mFirmwareUpdater(nullptr),
      mControllerNode(nullptr),
      mReporterNode(nullptr),
      mTaskIds(TaskId::AllTasks),
      mImageWidth(1280),
      mImageHeight(720),
      mLensFocus(8),
      mSensorPixelSize(4.2e-06f)
{
    QThreadPool::globalInstance()->setMaxThreadCount(20);
    init();
}

StereoCameraImpl::~StereoCameraImpl()
{
    if (mConnector) {
        mConnector->getProtocol()->unregisterBlockHandler(this);

        delete mFirmwareUpdater;
        delete mFrameReceiver;
        delete mFileReceiver;
        delete mMotionDataReceiver;

        mConnector->deleteLater();
    }

    if (mFileSender) delete mFileSender;
}

void StereoCameraImpl::init()
{
    mConnector  = new SATP::Connector();
    SATP::Protocol *protocol = mConnector->getProtocol();
    protocol->registerBlockHandler(this);

    mFrameReceiver = new FrameReceiver(protocol);

    mFileReceiver = new SATP::FileReceiver(protocol);
    mFileReceiver->registerReceiverHandler(this);
    mFileReceiver->setRecvFileDir(kAdasDownloadPath);

    mFirmwareUpdater = new FirmwareUpdater(protocol);
    connect(mFirmwareUpdater,
            SIGNAL(updateEvent(const QString&)),
            this,
            SLOT(onFirmwareUpdateEvent(const QString&)));

    mMotionDataReceiver = new MotionDataReceiver();
    connect(this, &StereoCameraImpl::cameraConnected, this, [this]{
        mMotionDataReceiver->init(getAddress());
    });

    connect(this, &StereoCameraImpl::cameraConnected, this, &StereoCameraImpl::initRemoteNode);

    mIsImuDataEnable = false;
}

bool StereoCameraImpl::isReplicaValid() const
{
    bool controllerReplicaValid = mControllerReplica && mControllerReplica->isReplicaValid();
    bool reporterReplicaValid = mReporterReplica && mReporterReplica->isReplicaValid();

    return controllerReplicaValid && reporterReplicaValid;
}

void StereoCameraImpl::connectTo(QString addr)
{
    mConnector->connectTo(addr);
}

SATP::Protocol *StereoCameraImpl::getProtocol()
{
    return mConnector->getProtocol();
}

void StereoCameraImpl::requestFrame(FrameHandler *handler, uint32_t frameIds)
{
    mFrameReceiver->requestFrame(handler, frameIds);
}

void StereoCameraImpl::requestMotionData(MotionDataHandler *handler)
{
    mMotionDataReceiver->requestMotionData(handler);
}

bool StereoCameraImpl::handleReceiveBlock(quint32 dataType, const char *block, int size)
{
    Q_UNUSED(dataType)
    Q_UNUSED(block)
    Q_UNUSED(size)
    return false;
}

void StereoCameraImpl::handleReady()
{
    emit cameraConnected();
}

void StereoCameraImpl::handleReset()
{
    emit cameraDisconnected();
}

void StereoCameraImpl::handleReceiveFile(const QString &fileName)
{
    if (fileName.contains("cameraData")) {
        QList<FrameHandler *> handlers = mFrameReceiver->getCameraHandlers();
        for (FrameHandler *handler : handlers) {
            (dynamic_cast<CameraHandler *>(handler))->handleCameraDataFile(fileName.toLocal8Bit().data());
        }
    }
}

void StereoCameraImpl::controlTasks()
{
    if (isReplicaValid()) {
        mControllerReplica->controlTasks(mTaskIds);
    }
}

void StereoCameraImpl::reconnect()
{
    mConnector->reconnect();
}

void StereoCameraImpl::disconnectServer()
{
    mConnector->disconnectServer();
}

bool StereoCameraImpl::isConnected()
{
    return mConnector->getProtocol()->isConnected();
}

QString StereoCameraImpl::getAddress()
{
    return mConnector->getHostName();
}

void StereoCameraImpl::enableTasks(uint32_t taskIds)
{
    mTaskIds = taskIds;
    if (isReplicaValid()) {
        controlTasks();
    }
}

void StereoCameraImpl::reboot(bool halt)
{
    if (isReplicaValid()) {
        mControllerReplica->reboot(halt);
    }
}

int StereoCameraImpl::updateFirmware(QString path)
{
    return mFirmwareUpdater->update(path);
}

double StereoCameraImpl::getUpdateProgress()
{
    return mFirmwareUpdater->getUpdateProgress();
}

int StereoCameraImpl::getUpdateStatus()
{
    return mFirmwareUpdater->getUpdateStatus();
}

void StereoCameraImpl::setReceiveDir(const QString &dir)
{
    mFileReceiver->setRecvFileDir(dir);
}

void StereoCameraImpl::onFirmwareUpdateEvent(const QString &event)
{
    QString e = event.toLower();
    Result result;
    if(e.contains("udp")){
        emit udpPackageReceived();
        return;
    }
    if(!e.contains("update")){
        return;
    }

    if(e.contains("success")){
        result.successed = true;
    }else if(e.contains("fail")){
        result.successed = false;
        result.warning = mFirmwareUpdater->getUpdateWarning();
    }
    emit updateFinished(result.successed);
    QList<FrameHandler *> handlers = mFrameReceiver->getCameraHandlers();
    for (FrameHandler *handler : handlers) {
        CameraHandler *camHandler = dynamic_cast<CameraHandler *>(handler);
        if(camHandler)camHandler->handleUpdateFinished(result);
    }
}

void StereoCameraImpl::initRemoteNode()
{
    if (!mControllerNode && !mReporterNode) {
        mControllerNode = new QRemoteObjectNode(this);
        QString controllerUrlStr = "tcp://" + getAddress() + ":" + QString::number(DeviceControllerReplica::NodePort::Default);
        mControllerNode->connectToNode(QUrl(controllerUrlStr));
        mControllerReplica = mControllerNode->acquire<DeviceControllerReplica>();
        mControllerReplica->waitForSource(3000);

        mReporterNode = new QRemoteObjectNode(this);
        QString reporterUrlStr = "tcp://" + getAddress() + ":" + QString::number(DeviceReporterReplica::NodePort::Default);
        mReporterNode->connectToNode(QUrl(reporterUrlStr));
        mReporterReplica = mReporterNode->acquire<DeviceReporterReplica>();
        mReporterReplica->waitForSource(3000);

        mFirmwareUpdater->acquireReplica(mReporterReplica);
        DeviceStatus::instance()->acquireReplica(mReporterReplica);
    }

    if (!isReplicaValid()) {
        mControllerReplica->waitForSource(3000);
        mReporterReplica->waitForSource(3000);
    } else {
        controlTasks();

        mImageWidth = mControllerReplica->imageWidth();
        mImageHeight = mControllerReplica->imageHeight();
        mLensFocus = mControllerReplica->lensFocus();
        mSensorPixelSize = mControllerReplica->sensorPixelSize() * 1e-06f;

        emit camImageWidthChanged(mImageWidth);
        emit camImageHeightChanged(mImageHeight);
        emit camLensFocusChanged(mLensFocus);
        emit camSensorPixelSizeChanged(mSensorPixelSize);
    }

    if (mIsImuDataEnable) {
        QTimer::singleShot(1000,this,[this]{
            enableMotionData(true);
        });
    }
}

bool StereoCameraImpl::requestStereoCameraParameters(StereoCalibrationParameters &params)
{
    if (!isReplicaValid()) {
        return false;
    }

    auto rpcData = mControllerReplica->stereoCameraParameters();

    if (rpcData.isNull()) {
        return false;
    }

    StereoCalibrationParameters *temp = reinterpret_cast<StereoCalibrationParameters*>(rpcData.data());

    params.focus = temp->focus;
    params.cx = temp->cx;
    params.cy = temp->cy;
    params.RRoll = temp->RRoll;
    params.RPitch = temp->RPitch;
    params.RYaw = temp->RYaw;
    params.Tx = temp->Tx;
    params.Ty = temp->Ty;
    params.Tz = temp->Tz;

    return true;
}

bool StereoCameraImpl::requestMonoLeftCameraParameters(MonoCalibrationParameters &params)
{
    if (!isReplicaValid()) {
        return false;
    }

    auto rpcData = mControllerReplica->monoLeftCameraParameters();

    if (rpcData.isNull()) {
        return false;
    }

    MonoCalibrationParameters *temp = reinterpret_cast<MonoCalibrationParameters*>(rpcData.data());

    params.fx = temp->fx;
    params.fy = temp->fy;
    params.cx = temp->cx;
    params.cy = temp->cy;
    params.k1 = temp->k1;
    params.k2 = temp->k2;
    params.k3 = temp->k3;
    params.p1 = temp->p1;
    params.p2 = temp->p2;

    return true;
}

bool StereoCameraImpl::requestMonoRightCameraParameters(MonoCalibrationParameters &params)
{
    if (!isReplicaValid()) {
        return false;
    }

    auto rpcData = mControllerReplica->monoRightCameraParameters();

    if (rpcData.isNull()) {
        return false;
    }

    MonoCalibrationParameters *temp = reinterpret_cast<MonoCalibrationParameters*>(rpcData.data());

    params.fx = temp->fx;
    params.fy = temp->fy;
    params.cx = temp->cx;
    params.cy = temp->cy;
    params.k1 = temp->k1;
    params.k2 = temp->k2;
    params.k3 = temp->k3;
    params.p1 = temp->p1;
    params.p2 = temp->p2;

    return true;
}

bool StereoCameraImpl::requestRotationMatrix(RotationMatrix &rotationMatrix)
{
    if (!isReplicaValid()) {
        return false;
    }

    auto rpcData = mControllerReplica->rotationMatrix();

    if (rpcData.isNull()) {
        return false;
    }

    RotationMatrix *temp = reinterpret_cast<RotationMatrix*>(rpcData.data());
    memcpy(rotationMatrix.real3DToImage, temp->real3DToImage, kNumReal3DToImage);
    memcpy(rotationMatrix.imageToReal3D, temp->imageToReal3D, kNumImageToReal3D);

    return true;
}

void StereoCameraImpl::enableMaxSendFrameInterval()
{
    mFrameReceiver->enableMaxSendFrameInterval();
}

bool StereoCameraImpl::setFrameRate(float rate)
{
    if (isReplicaValid()) {
        mControllerReplica->pushFrameRate(rate);
        return true;
    } else {
        return false;
    }
}

bool StereoCameraImpl::getFrameRate(float &rate)
{
    if (isReplicaValid()) {
        rate = mControllerReplica->frameRate();
        return true;
    } else {
        return false;
    }
}

bool StereoCameraImpl::getAmbientLight(int &lightness)
{
    if (isReplicaValid()) {
        lightness = mControllerReplica->ambientLight();
        return true;
    } else {
        return false;
    }
}

bool StereoCameraImpl::getSmudgeStatus(int &status)
{
    if (isReplicaValid()) {
        status = mControllerReplica->smudgeStatus();
        return true;
    }
    return false;
}

int StereoCameraImpl::getUpdateWarning()
{
    return mFirmwareUpdater->getUpdateWarning();
}

bool StereoCameraImpl::isDeviceHighTemperature()
{
    bool ret = getDeviceState().deviceHighTemperature()
            || mFirmwareUpdater->isDeviceHighTemperature();
    return ret;
}

void StereoCameraImpl::setImuParameter(const QString &name, int value)
{
    if (isReplicaValid()) {
        mControllerReplica->setImuParameter(name, value);
    }
}

int StereoCameraImpl::getImuParameter(const QString &name)
{
    if (isReplicaValid()) {
        return mControllerReplica->getImuParameter(name).returnValue();
    } else {
        return 0;
    }
}

bool StereoCameraImpl::sendFileToDevice(const QString &filePath)
{
    if (!mFileSender) {
        mFileSender = new SATP::FileSender(mConnector->getProtocol(), this);
    }
    if (!mConnector->isConnected()) return false;

    mFileSender->setAutoDelete(false);
    mFileSender->send(filePath);
    return true;
}

void StereoCameraImpl::handleSendFileFinished(const QString &fileName)
{
    emit fileSendFinished(fileName);
}

void StereoCameraImpl::enableMotionData(bool enable)
{
    if (isReplicaValid()) {
        mControllerReplica->enableMotionData(enable);
    }

    mIsImuDataEnable = enable;
}

const SEDeviceState &StereoCameraImpl::getDeviceState()
{
    return DeviceStatus::instance()->getDeviceState();
}
