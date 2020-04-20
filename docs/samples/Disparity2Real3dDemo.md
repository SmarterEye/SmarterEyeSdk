# Disparity2Real3dDemo

从深度图提取视差数据，并生成三维点云数据。
此Demo位于 `<sdk>/samples/Disparity2Real3dDemo` 下。

## 代码逻辑

* 与设备建立连接
* 请求并接收`深度图`数据
* 提取视差值前，需要从设备获取 `相机标定参数`
* 对收到的深度图数据进行处理，使用 [数据处理模块](api/image_utils) 提供的方法，提取视差值
* 把提取到的视差值数据保存到文件

## 参考代码片段

```C++
// 在handleRawFrame函数内，对接收到的图像进行处理
// 这里只处理深度图数据
void MyCameraHandler::processFrame(int frameId, char *image, uint32_t dataSize, int width, int height, int frameFormat)
{
    switch (frameId) {
    case FrameId::Disparity:
    {
        // 获取对应的深度图格式的BitNum
        int bitNum = DisparityConvertor::getDisparityBitNum(frameFormat);

        // 提取视差值前，需要从设备获取相机标定参数
        if (mIsCalibParamReady) {
            handleDisparityByLookupTable((unsigned char *)image, width, height, bitNum);
        } else {
            std::cout  << "calib params is not ready!!!" << std::endl;
        }
    }
        break;
    default:
        break;
    }
}

void MyCameraHandler::handleDisparityByLookupTable(unsigned char *image, int width, int height, int bitNum)
{
    std::cout << "width: " << width << ", height: " << height << std::endl;

    // generate X Y Z lookup table, only once is OK
    static float *lookUpTableX = new float[kDisparityCount*(int)pow(2, bitNum)*width];
    static float *lookUpTableY = new float[kDisparityCount*(int)pow(2, bitNum)*height];
    static float *lookUpTableZ = new float[kDisparityCount*(int)pow(2, bitNum)];
    if(!mIsLookupTableGenerated) {
        DisparityConvertor::generateLookUpTableX(width, bitNum, mStereoCalibrationParameters.Tx, mStereoCalibrationParameters.cx, lookUpTableX);
        DisparityConvertor::generateLookUpTableY(height, bitNum, mStereoCalibrationParameters.Tx, mStereoCalibrationParameters.cy, lookUpTableY);
        DisparityConvertor::generateLookUpTableZ(bitNum, mStereoCalibrationParameters.Tx, mStereoCalibrationParameters.focus, lookUpTableZ);
        mIsLookupTableGenerated = true;
    }

    // get X Y Z distance according to the lookup table
    float *xDistance = new float[width*height];
    DisparityConvertor::getWholeXDistanceByLookupTable(image, width, height, bitNum, lookUpTableX, xDistance);
    float *yDistance = new float[width*height];
    DisparityConvertor::getWholeYDistanceByLookupTable(image, width, height, bitNum, lookUpTableY, yDistance);
    float *zDistance = new float[width*height];
    DisparityConvertor::getWholeZDistanceByLookupTable(image, width, height, lookUpTableZ, zDistance);

    // get the 3D point cloud data, and save one to file for verification
    static int index = 0;
    index ++;
    FILE * fp = nullptr;
    if (index == 1) {
        fp = fopen(k3dByLookUpTableFilePath.data(), "wb+");
        if (!fp) {
            std::cout << k3dByLookUpTableFilePath << " file not open" << std::endl;
            return;
        }
    }
    for(int i = 0; i < width*height; i++) {
        float x, y, z;
        x = xDistance[i];
        y = yDistance[i];
        z = zDistance[i];
        if((fabs(x) > 200000.0f)||(fabs(y) > 200000.0f)||(fabs(z) > 200000.0f)) {
            x = 0.0f;
            y = 0.0f;
            z = 0.0f;
        }
        if (index == 1) {
            fprintf(fp, "%f %f %f %d\n", x, y, z, i);
        }
    }
    if (index == 1) {
        fclose(fp);
    }

    delete [] xDistance;
    delete [] yDistance;
    delete [] zDistance;
}
```
