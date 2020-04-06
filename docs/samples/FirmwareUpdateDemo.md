# FirmwareUpdateDemo

上传固件包，对设备进行固件升级。
此Demo位于 `<sdk>/samples/FirmwareUpdateDemo` 下。

!> 如需对设备进行固件升级，请向FAE人员获取对应的固件包，并确认设备型号和版本。

## 代码逻辑

* 与设备建立连接
* 连接上设备后，上传相应的固件包，等待升级完成

## 参考代码片段

```C++
int main(int argc, char *argv[])
{
    // 从命令行参数读取固件包的路径，别忘了包名
    if (argc < 2 || argv[1] == nullptr) {
        printf("please input the firmware package path. eg: ./FirmwareUpdateDemo SE_FirmWare_sm_2.0.1.tar.gz\n");
        return -1;
    }

    const char *packagePath = argv[1];

    std::fstream file;
    file.open(packagePath, std::ios::in);

    if (!file) {
        printf("%s, this firmware package does not exist!!!\n", packagePath);
        return -2;
    }

    StereoCamera *camera = StereoCamera::connect("192.168.1.251");
    MyCameraHandler *cameraHandlerA = new MyCameraHandler("camera A");

    camera->requestFrame(cameraHandlerA, FrameId::LeftCamera);

    while (!camera->isConnected()) {
        printf("connecting...\n");
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    // 待设备连接成功之后，上传对应的升级包
    printf("uploading %s\n", packagePath);
    camera->updateFirmware(packagePath);

    int c = 0;
    while (c!= 'x')
    {
        c = getchar();
    }
}
```