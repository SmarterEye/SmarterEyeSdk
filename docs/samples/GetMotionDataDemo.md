# GetMotionDataDemo

从设备获取IMU数据以及标定参数。
此Demo位于 `<sdk>/samples/GetMotionDataDemo` 下。

## 代码逻辑

* 与设备建立连接
* 请求并接收 `MotionData`，结构体定义在`motiondata.h`
* 请求数据时，需要调用`enableMotionData`来启用设备的`IMU`功能
* 打印出当前的`IMU`数据值

## 参考代码片段

```C++
int main(int argc, char *argv[])
{
    StereoCamera *cameraA = StereoCamera::connect("192.168.1.251");
    MyCameraHandler *cameraHandlerA = new MyCameraHandler("camera A");

    bool isConnected = false;
    while (!isConnected) {
        printf("connecting...\n");
        isConnected = cameraA->isConnected();
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    //get imu parameters
    std::cout << "accel range:" << cameraA->getImuAccelRange() << std::endl;
    std::cout << "rotation range:" << cameraA->getImuRotationRange() << std::endl;
    std::cout << "frequence::" << cameraA->getImuReadFrequence() << std::endl;

    //set imu parameters
    cameraA->setImuAccelRange(4); //2,4,8,16;default is 4
    cameraA->setImuRotationRange(500); //250,500,1000,2000;default is 500
    cameraA->setImuReadFrequence(100); //10-100Hz;default is 100Hz

    //request to get frames and motiondata
    cameraA->requestFrame(cameraHandlerA, FrameId::CalibLeftCamera);
    cameraA->enableTasks(TaskId::DisplayTask);
    cameraA->requestMotionData(cameraHandlerA);
    cameraA->enableMotionData(true);

    // ...
}

// ...

// MyCameraHandler 继承自 CameraHandler，并重写 handleMotionData，用于IMU数据到达时回调
void MyCameraHandler::handleMotionData(const MotionData *motionData)
{
    // get motiondata(imu) here...
    std::cout << "=======================" << std::endl;
    std::cout << "accel x: " << motionData->accelX << " g" << std::endl;
    std::cout << "accel y: " << motionData->accelY << " g" << std::endl;
    std::cout << "accel z: " << motionData->accelZ << " g" << std::endl;
    std::cout << "gyro x: " << motionData->gyroX << " deg/s" << std::endl;
    std::cout << "gyro y: " << motionData->gyroY << " deg/s" << std::endl;
    std::cout << "gyro z: " << motionData->gyroZ << " deg/s" << std::endl;
    std::cout << "time stamp: " << motionData->timestamp << " ms" << std::endl;
    std::cout << "=======================" << std::endl << std::endl;
}
```
