# GetCompoundDemo

获取双目相机左校准图与障碍物的复合数据。
此Demo位于 `<sdk>/samples/GetCompoundDemo` 下。

## 代码逻辑

* 与设备建立连接
* 请求并接收 `左校准图` 与 `障碍物` 的复合数据
* 提取障碍物信息，并在`左校准图`上进行绘制，绘制方法介绍见 [数据处理模块](api/image_utils?id=obstaclepainter-class) 。

## 参考代码片段

```C++
// 处理ID为Compound的帧数据
void MyCameraHandler::processFrame(int frameId, char *image, char *extended, int64_t time, int width, int height)
{
    switch (frameId) {
    case FrameId::Compound:
    {
        FrameDataExtHead *header = reinterpret_cast<FrameDataExtHead *>(extended);
        static unsigned char *rgbBuf = new unsigned char[width * height * 3];
        
        // 提取出障碍物数据，转换并绘制
        RoadwayPainter::imageGrayToRGB((unsigned char*)image, rgbBuf, width, height);
        ObstaclePainter::paintObstacle(header->data, rgbBuf, width, height, true, false);
    }
        break;
    default:
        break;
    }
}
```
