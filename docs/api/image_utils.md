# 数据处理模块

提供深度图数据的提取和转换，车道线绘制，障碍物绘制，以及`Yuv422`格式图像的转换方法。

!> 如果您使用的是我们提供的编译好的动态库，需要注意。此模块位于ImageUtils动态库中，需要用到该模块内的函数时，请在编译选项内链接该库。 以CMake为例，在CMakeLists.txt内，添加 `link_libraries(ImageUtils)`

## DisparityConvertor Class

Header: `#include "disparityconvertor.h"`

用于深度图的转换，以及从中提取需要的信息。

### Static Public Functions

> `void convertDisparity2FloatFormat(const unsigned char* src, int width, int height, int bitNum, float* dest)`  
用于将请求后获取到的视差图像数据进行整合转换，并存放在 `float` 类型变量中。

参数名称 | 类型 | 输入/输出 | 说明
:-: | :-: | :-: | :-: 
src | const unsigned char* | 输入 | 视差图像数据指针
dest | float* | 输出 | 转换后数据指针
width | int | 输入 | 图像宽度 （单位：像素）
height | int | 输入 | 图像高度 （单位：像素）
bitNum | int | 输入 | 视差数据中小数部分所占位数

> `void convertDisparity2RGB(const float* disparity, int width, int height, int minDisp, int maxDisp, unsigned char* rgbBuf)`  
将整合后的 `float` 类型视差数据进行伪彩色渲染。准确渲染一定范围内视差值，区间（minDsp，maxDisp）建议（0，45）

参数名称 | 类型 | 输入/输出 | 说明
:-: | :-: | :-: | :-: 
disparity | const float* | 输入 | 整合后的float类型视差数据指针
rgbBug | unsigned char* | 输出 | 用于存放经过伪彩色渲染的图像数据的指针
width | int | 输入 | 图像宽度 （单位：像素）
height | int | 输入 | 图像高度 （单位：像素）
minDisp | int | 输入 | 渲染视差值下限
maxDisp | int | 输入 | 渲染视差值上限

> `int getDisparityBitNum(int format)`  
根据接收到的视差数据帧格式，将视差数据中小数部分所占浮点数位数以 `int` 类型返回，该值用于视差数据的处理。

参数名称 | 类型 | 输入/输出 | 说明
:-: | :-: | :-: | :-: 
format | int | 输入 | 帧格式

> `void generateLookUpTableX(int width, int bitNum, float baseline, int cx, float* lookUpTableX)`  
生成视差数据与X方向距离值的对应关系映射表，该表将用于获取整幅图像所有像素点在相机坐标系X方向上与坐标原点的距离。

参数名称 | 类型 | 输入/输出 | 说明
:-: | :-: | :-: | :-: 
width | int | 输入 | 图像宽度 （单位：像素）
bitNum | int | 输入 | 视差数据中小数部分所占位数
baseline | float | 输入 | 相机参数：基线 （单位：毫米）
cx | int | 输入 | 相机参数：光学中心x方向坐标 （单位：像素）
lookUpTableX | float* | 输出 | 视差数据与相机坐标系中X方向距离值的对应关系映射表

> `void generateLookUpTableY(int height, int bitNum, float baseline, int cy, float* lookUpTableY)`  
生成视差数据与Y方向距离值的对应关系映射表，该表将用于获取整幅图像所有像素点在相机坐标系Y方向上与坐标原点的距离。

参数名称 | 类型 | 输入/输出 | 说明
:-: | :-: | :-: | :-: 
height | int | 输入 | 图像高度 （单位：像素）
bitNum | int | 输入 | 视差数据中小数部分所占位数
baseline | float | 输入 | 相机参数：基线 （单位：毫米）
cy | int | 输入 | 相机参数：光学中心y方向坐标 （单位：像素）
lookUpTableY | float* | 输出 | 视差数据与相机坐标系中Y方向距离值的对应关系映射表

> `void generateLookUpTableZ(int bitNum, float baseline, float focus, float* lookUpTableZ)`  
生成视差数据与Z方向距离值的对应关系映射表，该表将用于获取整幅图像所有像素点在相机坐标系Z方向上与坐标原点的距离。

参数名称 | 类型 | 输入/输出 | 说明
:-: | :-: | :-: | :-: 
bitNum | int | 输入 | 视差数据中小数部分所占位数
baseline | float | 输入 | 相机参数：基线 （单位：毫米）
foucs | float | 输入 | 相机参数：焦距 （单位：像素）
lookUpTableZ | float* | 输出 | 视差数据与相机坐标系中Z方向距离值的对应关系映射表

> `void getWholeXDistanceByLookupTable(const unsigned char* src, int width, int height, int bitNum, float* lookUpTableX, float* distanceX)`  
查表获取整幅图像所有像素点在相机坐标系X方向上与坐标原点的距离（单位：毫米）。

参数名称 | 类型 | 输入/输出 | 说明
:-: | :-: | :-: | :-: 
src | const unsigned char* | 输入 | 视差图像数据
width | int | 输入 | 图像宽度 （单位：像素）
height | int | 输入 | 图像高度 （单位：像素）
bitNum | int | 输入 | 视差数据中小数部分所占位数
lookUpTableX | float* | 输入 | 视差数据与相机坐标系中X方向距离值的对应关系映射表
distanceX | float* | 输出 | 全幅图像在X方向距离值指针

> `void getWholeYDistanceByLookupTable(const unsigned char* src, int width, int height, int bitNum, float* lookUpTableY, float* distanceY)`  
查表获取整幅图像所有像素点在相机坐标系Y方向上与坐标原点的距离（单位：毫米）。

参数名称 | 类型 | 输入/输出 | 说明
:-: | :-: | :-: | :-: 
src | const unsigned char* | 输入 | 视差图像数据
width | int | 输入 | 图像宽度 （单位：像素）
height | int | 输入 | 图像高度 （单位：像素）
bitNum | int | 输入 | 视差数据中小数部分所占位数
lookUpTableY | float* | 输入 | 视差数据与相机坐标系中Y方向距离值的对应关系映射表
distanceY | float* | 输出 | 全幅图像在Y方向距离值指针

> `void getWholeZDistanceByLookupTable(const unsigned char* src, int width, int height, float* lookUpTableZ, float* distanceZ)`  
查表获取整幅图像所有像素点在相机坐标系Z方向上与坐标原点的距离（单位：毫米）。

参数名称 | 类型 | 输入/输出 | 说明
:-: | :-: | :-: | :-: 
src | const unsigned char* | 输入 | 视差图像数据
width | int | 输入 |  图像宽度 （单位：像素）
height | int | 输入 | 图像高度 （单位：像素）
lookUpTableZ | float* | 输入 | 视差数据与相机坐标系中Z方向距离值的对应关系映射表
distanceZ | float* | 输出 | 全幅图像在Z方向距离值指针

> `float getPointDisparityValue(const unsigned char *src, int width, int height, int bitNum, int posX, int posY)`  
根据像素点在视差图像中位置坐标（posX，posY），获取该点对应的视差数据值，以 `float` 类型返回（单位：像素）。

参数名称 | 类型 | 输入/输出 | 说明
:-: | :-: | :-: | :-: 
src | const unsigned char* | 输入 | 视差图像数据
width | int | 输入 | 图像宽度 （单位：像素）
height | int | 输入 | 图像高度 （单位：像素）
bitNum | int | 输入 | 视差数据中小数部分所占位数
posX | int | 输入 | 像素点在全幅图像X方向坐标 （单位：像素）
posY | int | 输入 | 像素点在全幅图像Y方向坐标 （单位：像素）

> `void getPointXYZDistance(const unsigned char *src, int width, int height, int bitNum,f loat baseline, float focus, int cx, int cy, int posX, int posY, float &xDistance, float &yDistance, float &zDistance)`  
获取像素点在X、Y、Z三个方向上与坐标原点（左相机镜头中心）的距离（单位：毫米）。

参数名称 | 类型 | 输入/输出 | 说明
:-: | :-: | :-: | :-: 
src | const unsigned char* | 输入 | 视差图像数据
width | int | 输入 | 图像宽度 （单位：像素）
height | int | 输入 | 图像高度 （单位：像素）
bitNum | int | 输入 | 视差数据中小数部分所占位数
baseline | float | 输入 | 相机参数：基线 （单位：毫米）
focus | float | 输入 | 相机参数：焦距 （单位：像素）
cx | int | 输入 | 相机参数：光学中心x方向坐标 （单位：像素）
cy | int | 输入 | 相机参数：光学中心y方向坐标 （单位：像素）
posX | int | 输入 | 像素点在全幅图像x方向坐标 （单位：像素）
posY | int | 输入 | 像素点在全幅图像y方向坐标 （单位：像素）
xDistance | float | 输出 | 像素点在X方向距离
yDistance | float | 输出 | 像素点在Y方向距离
zDistance | float | 输出 | 像素点在Z方向距离

> `void getWholeXDistance(const float* disparity, int width, int height, float baseline, int cx, float* xDistance)`  
获取整幅图像所有像素点在相机坐标系X方向上与坐标原点的距离（单位：毫米）。

参数名称 | 类型 | 输入/输出 | 说明
:-: | :-: | :-: | :-: 
disparity | const float* | 输入 | 整合后的全幅视差数据
width | int | 输入 | 图像宽度 （单位：像素）
height | int | 输入 | 图像高度 （单位：像素）
baseline | float | 输入 | 相机参数：基线 （单位：毫米）
cx | int | 输入 | 相机参数：光学中心x方向坐标 （单位：像素）
xDistance | float* | 输出 | 全幅图像在X方向距离值

> `void DisparityConvertor::getWholeYDistance(const float* disparity, int width, int height, float baseline, int cy, float* yDistance)`  
获取整幅图像所有像素点在相机坐标系Y方向上与坐标原点的距离（单位：毫米）。

参数名称 | 类型 | 输入/输出 | 说明
:-: | :-: | :-: 
disparity | const float* | 输入 | 整合后的全幅视差数据
width | int | 输入 | 图像宽度 （单位：像素）
height | int | 输入 | 图像高度 （单位：像素）
baseline | float | 输入 | 相机参数：基线 （单位：毫米）
cy | int | 输入 | 相机参数：光学中心y方向坐标 （单位：像素）
yDistance | float* | 输出 | 全幅图像在Y方向距离值

> `void getWholeZDistance(const float* disparity, int width, int height, float baseline, float focus, float* zDistance)`  
获取整幅图像所有像素点在相机坐标系Z方向上与坐标原点的距离（单位：毫米）。

参数名称 | 类型 | 输入/输出 | 说明
:-: | :-: | :-: | :-: 
disparity | const float* | 输入 | 整合后的全幅视差数据
width | int | 输入 | 图像宽度 （单位：像素）
height | int | 输入 | 图像高度 （单位：像素）
baseline | float | 输入 | 相机参数：基线 （单位：毫米）
focus | float | 输入 | 相机参数：焦距 （单位：像素）
zDistance | float* | 输出 | 全幅图像在Z方向距离值指针


## RoadwayPainter Class

Header: `#include "roadwaypainter.h"`

提供车道线数据绘制的方法。

### Static Public Functions

> `void imageGrayToRGB(const unsigned char *gray, unsigned char *rgb, int _width, int _height)`  
将单通道灰度图转三通道RGB，用于后续障碍物或车道线的彩色效果。

参数名称 | 类型 | 输入/输出 |  说明
:-: | :-: | :-: | :-:
gray | const unsigned char* | 输入 | 单通道灰度图
rgb | const unsigned char* | 输出 | 三通道RGB
width | int | 输入 | 图像宽度 （单位：像素）
height | int | 输入 | 图像高度 （单位：像素）

> `bool paintRoadway(void *_roadwayParam, unsigned char * _rgbImageData, int _width_, int _height, bool maskMode = false)`  
该函数会融合车道线信息，修改三通道RGB图像数据，以达到将车道线信息绘制到图像上的目的。调用该函数后，对 `_rgbImageData` 数据进行后续处理即可。返回值为 `bool` 类型，代表图像内是否有车道线被识别。

参数名称 | 类型 | 输入/输出 |  说明
:-: | :-: | :-: | :-:
_roadwayParam | void* | 输入 | 车道线处理结果数据
_rgbImageData | unsigned char* | 输出 | 三通道RGB图像数据
_width | int  | 输入 | 图像宽度 （单位：像素）
_height | int | 输入 | 图像高度 （单位：像素）
maskMode | bool | 输入 | 是否采用混色方式绘制 （true：否  false：是）


## ObstaclePainter Class

Header: `#include "obstaclepainter.h"`

提供障碍物数据绘制的方法。

### Static Public Functions

> `bool paintObstacle(void * _obstacleParam, unsigned char * _rgbImageData, int _width, int _height, bool showDetials, bool singleObs)`  
该函数会融合障碍物信息，修改三通道RGB图像数据，以达到将障碍物信息绘制到图像上的目的。调用该函数后，对 `_rgbImageData` 数据进行后续处理即可。返回值为 `bool` 类型，绘制结果是否成功。

参数名称 | 类型 | 输入/输出 |  说明
:-: | :-: | :-: | :-:
_obstacleParam | void* | 输入 | 障碍物处理结果数据
_rgbImageData | unsigned char* | 输出 | 三通道RGB图像数据
_width | int | 输入 | 图像宽度 （单位：像素）
_height | int | 输入 | 图像高度 （单位：像素）
showDetials | bool | 输入 |  是否显示障碍物详细信息 
singleObs | bool | 输入 | 是否仅绘制预警区域内最近障碍物

## YuvToRGB Class

Header: `#include "yuv2rgb.h"`

通过判断 `RawImageFrame` 结构中 `format 成员`，确定图像数据所使用的格式，定义见frameformat.h。

### Static Public Functions

> `char* YCbYCr2Rgb(const unsigned char* src, char* dest, int width, int height)`  
将`YUV422`格式图像数据转换为三通道RGB数据。

参数名称 | 类型 | 输入/输出 |  说明
:-: | :-: | :-: | :-:
src | const unsigned char* | 输入 | YUV422图像数据
dest | char* | 输出 | 三通道RGB图像数据
width | int | 输入 | 图像宽度 （单位：像素）
height | int | 输入 | 图像高度 （单位：像素）

> `char* YCbYCrPlannar2Rgb(const unsigned char* src, char* dest, int width, int height)`  
将`YUV422planar`格式图像数据转换为三通道RGB数据。

参数名称 | 类型 | 输入/输出 |  说明
:-: | :-: | :-: | :-:
src | const unsigned char* | 输入 | YUV422planar图像数据
dest | char* | 输出 | 三通道RGB图像数据
width | int | 输入 | 图像宽度 （单位：像素）
height | int | 输入 | 图像高度 （单位：像素）





