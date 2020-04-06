# GetLaneExtDemo

获取双目相机左校准图与车道线的复合数据。
此Demo位于 `<sdk>/samples/GetLaneExtDemo` 下。

## 代码逻辑

* 与设备建立连接
* 请求并接收 `左校准图` 与 `车道线` 的复合数据
* 提取车道线信息，并在`左校准图`上进行绘制，绘制方法介绍见 [数据处理模块](api/image_utils?id=roadwaypainter-class) 。

## 参考代码片段

```C++
// 处理ID为LaneExt的帧数据
void MyCameraHandler::processFrame(int frameId, char *image, char *extended, int64_t time, int width, int height)
{
    switch (frameId) {
    case FrameId::LaneExt:
    {
        // 提取车道线数据
        //extended is LaneInfo of this frame,you can use it to paintRoadway or print.
        FrameDataExtHead *header = reinterpret_cast<FrameDataExtHead *>(extended);
        LdwDataPack *ldwDataPack = (LdwDataPack *)header->data;
        std::cout << "ldw degree is: " << ldwDataPack->roadway.left_Lane.left_Boundary.degree << std::endl;

        static unsigned char *rgbBuf = new unsigned char[width * height * 3];
        
        // 调用相关函数进行绘制
        RoadwayPainter::imageGrayToRGB((unsigned char*)image, rgbBuf, width, height);
        mIsLaneDetected = RoadwayPainter::paintRoadway(header->data, rgbBuf, width, height);
    }
        break;
    default:
        break;
    }
}
```
