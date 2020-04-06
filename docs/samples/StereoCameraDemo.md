# StereoCameraDemo

关于SDK的一些基础用法。
此Demo位于 `<sdk>/samples/StereoCameraDemo` 下。

## 代码逻辑

* 与设备建立连接
* 请求并接收所需的图像数据
* 处理数据前，从设备获取所需的参数信息
* 接收图像数据的对象里面，对收到的数据进行处理

## 参考代码片段

> main函数

```C++
int main(int argc, char *argv[])
{
    // 首先建立起与设备的网络连接
    StereoCamera *cameraA = StereoCamera::connect("192.168.1.251");

    // 建立一个CameraHandler对象，用来接收设备发送过来的图像数据
    MyCameraHandler *cameraHandlerA = new MyCameraHandler("camera A");

    // 启用设备上需要运行的功能，这里是启用的是：障碍物提取、车道线检测、传图功能。
    // 这些功能默认是开启的，不执行这一步也可以。
    cameraA->enableTasks(TaskId::ObstacleTask | TaskId::LaneTask | TaskId::DisplayTask);

    // 向设备请求所需的图像帧类型，并指定接收的对象
    cameraA->requestFrame(cameraHandlerA, FrameId::Disparity);

    // 如果同时连接了多台设备，可以建立多个连接和多个接收对象。
    //    StereoCamera *cameraB = StereoCamera::connect("192.168.20.100");
    //    MyCameraHandler *cameraHandlerB = new MyCameraHandler("camera B");
    //    cameraB->requestFrame(cameraHandlerB, FrameId::CalibRightCamera);

    // 循环等待以防程序退出
    int c = 0;
    while (c!= 'x')
    {
        switch (c)
        {
        case 'f':
        {
            // 处理深度图信息需要用到双目相机的标定参数，这里向设备请求获双目标定数据，用于后续的操作
            StereoCalibrationParameters params;
            if (cameraA->requestStereoCameraParameters(params)) {
                cameraHandlerA->setStereoCalibParams(params);
            } else {
                std::cout << "Stereo camera parameters request failed." << std::endl;
            }
        }
            break;
        case 'r':
        {
            // 向设备请求旋转矩阵的数据
            RotationMatrix rotationMatrix;
            if (cameraA->requestRotationMatrix(rotationMatrix)) {
                cameraHandlerA->setRotationMatrix(rotationMatrix);
            } else {
                std::cout << "Rotation matrix request failed." << std::endl;
            }
        }
            break;
        case 'u':
        {
            // 如果需要对设备进行固件升级，则执行这一步，从命令行获取固件包的路径，上传给设备进行升级
            std::cout << "Input the path of your local firmware package: " << std::endl;
            std::string packagePath;
            std::cin >> packagePath;
            cameraA->updateFirmware(packagePath.data());
        }
            break;
        case 'l':
        {
            // 从设备获取外部环境光亮度
            int light = -1;
            bool result = false;
            result = cameraA->getAmbientLight(light);
            if(result){
                switch(light){
                case 1:
                    std::cout<<"day"<<std::endl;
                    break;
                case 2:
                    std::cout<<"dusk"<<std::endl;
                    break;
                case 3:
                    std::cout<<"night"<<std::endl;
                    break;
                default:
                    std::cout<<"unknown"<<std::endl;
                }
            }
        }
            break;
        case 'z':
        {
            // 断开与设备的网络连接
            cameraA->disconnectFromServer();
        }
            break;
        default:
            break;
        }
        c = getchar();
    }
}
```

> CameraHandler class
```C++
// 继承CameraHandler并重写虚函数，处理接收的图像和升级结果
class MyCameraHandler: public CameraHandler
{
public:
    MyCameraHandler(std::string name);
    void handleRawFrame(const RawImageFrame *rawFrame);
    void handleUpdateFinished(Result result);
    
    // ...
}

// 当收到图像，该函数在接收线程内被调用
void MyCameraHandler::handleRawFrame(const RawImageFrame *rawFrame)
{
    // 可根据不同的图像，进行不同的处理动作
    processFrame(rawFrame->frameId, (char*)rawFrame + sizeof(RawImageFrame),
                 rawFrame->dataSize, rawFrame->width, rawFrame->height, rawFrame->format);
}

// 设备固件升级完成后，不论成功与否，都会被调用，返回升级结果
void MyCameraHandler::handleUpdateFinished(Result result)
{
    mUpgradeResult = result;
    if (!mUpgradeResult.successed) {
        std::cout <<"Update failed with warning:"<<mUpgradeResult.warning << std::endl;
        std::cout << "If you want to update firmware, please press 'u' to try again !!! " << std::endl;
    }
}
```
