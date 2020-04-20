# StereoCamera

SDK的主要接口，负责与双目相机建立网络连接，数据请求，设备配置等工作。

!> 如果您使用的是我们提供的编译好的动态库，需要注意。此模块位于StereoCamera库中，需要用到该模块内的函数时，请在编译选项内链接该库。 以CMake为例，在CMakeLists.txt内，添加 `link_libraries(StereoCamera)`  

## 使用示例

```C++
int main(int argc, char *argv[])
{
    StereoCamera *cameraA = StereoCamera::connect("192.168.1.251");
    // ...
}
```

## StereoCamera Class

Header: `#include "stereocamera.h"`

### Static Public Functions

> `static StereoCamera *connect(const char *addr)`  
SDK最主要的函数，与指定IP地址的双目相机建立连接，返回一个实例化后的StereoCamera对象指针。

### Public Functions

#### 设备连接

> `void disconnectFromServer()`  
 断开与设备的网络连接。

> `bool isConnected()`  
返回与设备的连接状态。true为已连接，false为未连接。

> `SATP::Protocol *getProtocol()`  
获取网络连接协议，SATP::Protocol为中科慧眼封装的TCP/IP应用层协议。

#### 设备操作

> `void reboot(bool halt = false)`  
给设备发送 关机/重启 命令，参数halt为true表示关机，false表示重启设备。

> `int updateFirmware(const char* path)`  
上传指定升级包对设备进行固件升级。参数path为升级包绝对路径带文件名，返回值为上传升级包的结果，具体描述如下。  

 | 返回值 | 描述 |
 | :---: | :---: |
 | 0      | 正常    |
 | -1     | 无效文件 |
 | -2     | 服务忙（有文件正在上传或正在升级中） |
 | -3     | 打开文件失败     |
 | 1 ~ 9  | 相机相关异常（参考相机类故障码）    |
 | 19      | 设备高温    |

 > `double getUpgradeProgress()`  
 获取设备固件升级的进度，0.0 ~ 1.0。

> `void setFileReceiveDir(const char *dir)`  
设置从设备接收的文件存放到本地的路径。参数dir为目录，要求目录必须存在且有读写权限。

> `void enableTasks(uint32_t taskIds)`  
启用设备上需要运行的功能(如有)，参数taskIds为枚举类型，定义见 taskiddef.h。

#### 数据请求

> `void requestFrame(FrameHandler *frameHandler, uint32_t frameIds)`  
向设备请求所需的图像帧数据，根据不同的frameId，请求不同的图像，数据到达会在`frameHandler`回调。

> `bool requestStereoCameraParameters(StereoCalibrationParameters &params)`  
请求双目相机标定参数。

> `bool requestStereoCameraParameters(StereoCalibrationParameters &params)`  
请求双目相机标定参数。

> `bool requestMonoLeftCameraParameters(MonoCalibrationParameters &params)`  
请求单目左相机标定参数。

> `bool requestMonoRightCameraParameters(MonoCalibrationParameters &params)`  
请求单目右相机标定参数。

> `bool requestRotationMatrix(RotationMatrix &matrix)`  
请求旋转矩阵数据。

### IMU数据

> `void requestMotionData(MotionDataHandler *motionDataHandler)`  
向设备请求`IMU`数据，数据到达会在`motionDataHandler`回调。


> `void setImuAccelRange(int value)`  
> `void setImuRotationRange(int value)`  
> `void setImuReadFrequence(int value)`  
设置设备当前`IMU`参数。

> `int getImuAccelRange()`  
> `int getImuRotationRange()`  
> `int getImuReadFrequence()`  
从设备获取`IMU`参数。

### 相机配置

> `void enableMaxSendFrameInterval()`  
让双目相机以最大输出帧率输出图像。

> `bool setFrameRate(float rate)`  
指定要设置的相机帧率，参数rate为 fps，如 12.5。

> `bool getFrameRate(float &rate)`  
获取双目相机当前工作的帧率，参数rate是引用类型，为传入传出参数。

> `bool getAmbientLight(int &lightness)`  
获取外部环境光亮度。

> `bool getSmudgeStatus(int &status)`  
获取相机镜头脏污状态。

### 特殊函数

> `void invokeInLoopThread(std::function<void()> method)`  
在Qt EventLoop线程里面执行 传入的函数对象。
