# 中科慧眼双目摄像头灵动版常见问题

## 相机使用问题

- **Ubuntu应该怎样配置网口直接连接笔记本呢？**
> 地址处填写与ADAS设备同网段不同IP即可。  
![connection](..\assets\connection.png)

- **FPGA时间是怎样得到的呢？**
> FPGA时间是设备系统时间。
> * 如果没有给ADAS设备外接GPS模块，时间戳即为没有同步的unix时间。  
> * 如果外接了GPS模块，且信号良好可以完成授时，时间戳即为同步了世界时间的unix时间。  

- **InfiniteCG-8在Ubuntu 16.04上运行StereoCameraDemo时，卡在`camera connected got update progress is: 0`，正常吗？**
> 是正常的，连接成功。

- **深度图可以显示为彩色吗？像SmartEye工具显示的那样。**
> 可以，深度图的显示可以通过修改demo来变化，可以显示为灰度图，也可以做伪彩色渲染。  
在`DisplayFramesDemo/framemonitor.cpp`中，`loadFrameData2Mat()`函数，case Disparity16时，把注释的几行释放出来，下面的三行注掉就可以了。
![Display](..\assets\Display.jpg)


## SDK编译环境问题

- **SDK目前支持哪些编译器？**
> * Windows：Visual Studio 2017(vc15 x64)
> * Linux: GNU G++/GCC (Ubuntu 16.04及以上版本)


- **SDK目前依赖哪些库？**
> 目前SDK依赖的库（框架）主要是Qt，samples下有pcl和opencv使用的示例，wrapper目录下有对ros的封装程序。
> 由于某些特性，对Qt的版本有硬性要求，需要安装的Qt版本范围是`5.12.0 - 5.12.3`。原因见https://doc.qt.io/qt-5/qtremoteobjects-compatibility.html

- **SDK下的示例samples编译失败**
> samples目录下提供SDK的使用示例，sdk源码下载后，需要先把SDK的库编译安装完成后，才能编译samples下的程序；
>
> PointCloudDemo编译失败，有可能是系统安装的pcl库依赖的Qt与SDK依赖的Qt版本不一致引起的冲突，参考[此处](setup/env?id=pcl)；
>
> DisplayFramesDemo编译失败，可能是OpenCV依赖的Qt版本与SDK依赖Qt版本不一致引起的冲突，如果是安装ros时依赖的安装的OpenCV，或使用`apt-get install libopencv-dev`，可能会附带安装其它版本Qt从而与SDK引起冲突。这里需要再另外安装OpenCV，并且该OpenCV在编译时不依赖Qt进行编译。最后在samples/find_opencv.cmake下，指定安装的OpenCV的路径，如 `set(OpenCV_DIR /usr/local/lib/cmake/OpenCV）`。


## ROS Wrapper编译问题

- **直接catkin_make报错，找不到头文件。**
> 在Ubuntu 16.04下有此问题，可能是编译器默认没开启C++11支持，Ubuntu 18.04及以上版本则正常。
> ![nosuchfile](..\assets\nosuchfile.jpg)
srv文件没有生成对应的头文件，原因是编译器版本的选项不对，在`SmarterEyeSdk/wrapper/ros/src/zkhy_stereo_d/CMakeLists.txt`中配置一下c++11的编译选项即可。  
![c++](..\assets\c++.png)

- **Qt库link失败**
> ![make](..\assets\make.png)
原因是编译链接时，没有找到Qt5.12。  
需要将安装的qt所依赖的库目录，添加到环境变量LD_LIBRARY_PATH下。  
eg：export LD_LIBRARY_PATH=/your/path/Qt5.12.2/5.12.2/gcc_64/lib:$LD_LIBRARY_PATH   