# GetCameraParamDemo

获取双目相机的标定参数。
此Demo位于 `<sdk>/samples/GetCameraParamDemo` 下。

?> 双目相机标定参数的相机介绍，请参考 [双目相机标定参数](algorithm/calibration?id=%e6%a0%87%e5%ae%9a%e5%8f%82%e6%95%b0%e8%af%b4%e6%98%8e) 。

## 代码逻辑

* 与设备建立连接
* 连接上设备后，获取双目相机的标定参数

## 参考代码片段

```C++
void dumpCameraParams(const StereoCalibrationParameters &params)
{
    printf("************ Camera params ************\n");
    printf("Focus: %e pixel\n", params.focus);
    printf("Optical center X: %e pixel\n", params.cx);
    printf("Optical center Y: %e pixel\n", params.cy);
    printf("R-vector roll: %e rad\n", params.RRoll);
    printf("R-vector pitch: %e rad\n", params.RPitch);
    printf("R-vector yaw: %e rad\n", params.RYaw);
    printf("Translation x: %e mm\n", params.Tx);
    printf("Translation y: %e mm\n", params.Ty);
    printf("Translation z: %e mm\n", params.Tz);
    printf("**************************************\n");
}

int main(int argc, char *argv[])
{
    StereoCamera *camera = StereoCamera::connect("192.168.1.251");

    while (!camera->isConnected()) {
        printf("connecting...\n");
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    // 等待设备数据同步
    std::this_thread::sleep_for(std::chrono::seconds(3));

    // 获取标定参数
    StereoCalibrationParameters params;
    camera->requestStereoCameraParameters(params);

    dumpCameraParams(params);

    int c = 0;
    while (c!= 'x')
    {
        c = getchar();
    }
}
```