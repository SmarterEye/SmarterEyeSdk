# 数据接收接口

用于接收从设备获取的图像等数据。

## 接口能力

提供图像数据、算法数据、相机标定数据等。  
图像数据及算法数据通用结构体`RawImageFrame`，声明在头文件`satpext.h`中。

```C++
struct RawImageFrame
{
    uint16_t frameId;           //数据类型ID
    int64_t  time;              //时间戳
    uint16_t index;             //索引（保留字段，默认为0）
    uint16_t format;            //帧格式
    uint16_t width;             //图像宽度
    uint16_t height;            //图像高度
    uint32_t speed;             //行驶速度
    uint32_t dataSize;          //图像数据大小
    uint8_t  image[0];          //图像数据首地址
};
```

其中 `frameId` 为区别不同图像数据的唯一标识，它的定义如下。

```C++
struct FrameId
{
    enum Enumeration {
        NotUsed              = 0,
        LeftCamera           = 1 << 0,      //左相机原图
        RightCamera          = 1 << 1,      //右相机原图
        CalibLeftCamera      = 1 << 2,      //左相机校准图
        CalibRightCamera     = 1 << 3,      //右相机校准图
        DisparityDSBak       = 1 << 4,
        DisparityUV          = 1 << 5,
        Disparity            = 1 << 6,      //视差数据
        DisparityPlus        = 1 << 7,
        DisparityDS          = 1 << 8,      //降采样视差数据
        Lane                 = 1 << 9,      //车道线数据
        Obstacle             = 1 << 10,     //障碍物数据
        Compound             = 1 << 11,     //左校准图与障碍物数据的复合数据
        LDownSample          = 1 << 12,     //降采样左校准图
        RDownSample          = 1 << 13,     //降采样右校准图
        LaneExt              = 1 << 14,     //左校准图与车道线数据的复合数据
    };
};
```

`相机原图` 与 `相机校准图`的关系为：校准图是将标定数据作用于原图而得到的。
目前出货设备均已完成标定流程，处于`正常模式` ，在该模式下，提供的是`左右相机校准图`.

!> 1. FrameId::LeftCamera 不可和 FrameId::CalibLeftCamera同时获取。  
2. FrameId::RightCamera 不可和 FrameId::CalibRightCamera同时获取。  
3. FrameId::Lane不可和 FrameId::LaneExt同时获取。  
4. FrameId::Obstacle不可和 FrameId::Compound同时获取。  
5. FrameId::Disparity即视差数据左边有104 pixel宽度的空白，为匹配后所填充。

## FrameHandler Class

Header: `#include "framehandler.h"`

Inherited By: `CameraHandler`

### Interfaces

> `virtual void handleRawFrame(const RawImageFrame *rawFrame) = 0`  
相机图像接收的接口，参数rawFrame为从相机获取到的图像及相关数据。

## MotionDataHandler Class

Header: `#include "motiondatahandler.h"`

Inherited By: `CameraHandler`

### Interfaces

> `virtual void handleMotionData(const MotionData *motionData) = 0`  
IMU数据接收的接口，参数motionData为从相机获取到的IMU数据。

## CameraHandler Class

Header: `#include "camerahandler.h"`

Inherits: `FrameHandler`, `MotionDataHandler`

### Public Functions

> `virtual void handleCameraDataFile(const char *path)`  
在接收到请求的标定文件后，此函数会被调用。参数path为标定文件存放路径。

> `virtual void handleStereoCameraParam(float focus, float baseline, float pixelSize, int opticalCenterX, int opticalCenterY)`  
在获取到相机参数后，此函数会被调用。参数列表为获取到的相机参数具体数值。

> `virtual void handleUpdateFinished(Result result)`  
在设备升级完成后，此函数会被调用。参数result为返回的升级结果。

?> 需要自定义一个类并继承CameraHandler，重写虚函数，当条件触发后，其虚函数会被调用。

### 使用示例

```C++
class MyCameraHandler: public CameraHandler
{
public:
    MyCameraHandler(std::string name) {}
    void handleRawFrame(const RawImageFrame *rawFrame) {} // 图像接收入口，在SATP协议内的线程回调，无需主动调用
    void handleMotionData(const MotionData *motionData) {} // 请求IMU数据后，MotionData数据入口
    void handleUpdateFinished(Result result) {}           // 固件升级结果通知，升级结束后，返回升级结果，无需主动调用
};

int main(int argc, char *argv[])
{
    StereoCamera *cameraA = StereoCamera::connect("192.168.1.251");
    MyCameraHandler *cameraHandlerA = new MyCameraHandler("camera A");
    cameraA->requestFrame(cameraHandlerA, FrameId::Disparity);
    cameraA->requestMotionData(cameraHandlerA);
    cameraA->enableMotionData(true);
    // ...
}
```
