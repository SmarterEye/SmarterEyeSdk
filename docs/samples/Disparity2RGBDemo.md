# Disparity2RGBDemo

深度图的伪彩色渲染。
此Demo位于 `<sdk>/samples/Disparity2RGBDemo` 下。

## 代码逻辑

* 与设备建立连接
* 请求并接收`深度图`数据
* 对收到的深度图数据进行处理，使用 [数据处理模块](api/image_utils) 提供的方法，进行渲染

## 参考代码片段

```C++
static const int kMinValidDisparityValue = 0;
static const int kMaxValidDisparityValue = 45;

// 在handleRawFrame函数内，对接收到的图像进行处理
// 这里处理深度图，并转换成RGB图像
void MyCameraHandler::processFrame(int frameId, const unsigned char *image,int64_t time, int width, int height, int frameFormat)
{
    switch (frameId) {
    case FrameId::Disparity:
    {
        int bitNum = DisparityConvertor::getDisparityBitNum(frameFormat);
        static unsigned char *rgbBuf = new unsigned char[width * height * 3];
        static float *floatData = new float[width * height];

        DisparityConvertor::convertDisparity2FloatFormat(image, width, height, bitNum, floatData);
        DisparityConvertor::convertDisparity2RGB(floatData, width, height,
                                                 kMinValidDisparityValue, kMaxValidDisparityValue, rgbBuf);
    }
        break;
    default:
        break;
    }
}
```
