# 中科慧眼双目摄像头灵动版常见问题

- ## Ubuntu应该怎样配置网口直接连接笔记本呢？
地址处填写与ADAS设备同网段不同IP即可。  
![connection](..\assets\connection.png)

- ## FPGA时间是怎样得到的呢？
FPGA时间是设备系统时间。  
- 如果没有给ADAS设备外接GPS模块，时间戳即为没有同步的unix时间。  
- 如果外接了GPS模块，且信号良好可以完成授时，时间戳即为同步了世界时间的unix时间。  

- ## InfiniteCG-8在Ubuntu 16.04上运行StereoCameraDemo时，卡在`camera connected got update progress is: 0`，正常吗？
是正常的，连接成功。

- ## 深度图可以显示为彩色吗？像SmartEye工具显示的那样。
可以，深度图的显示可以通过修改demo来变化，可以显示为灰度图，也可以做伪彩色渲染。  
在`DisplayFramesDemo/framemonitor.cpp`中，`loadFrameData2Mat()`函数，case Disparity16时，把注释的几行释放出来，下面的三行注掉就可以了。
![Display](..\assets\Display.jpg)

- ## ROS Wrapper 编译问题
- ### 直接catkin_make报错，找不到头文件
![nosuchfile](..\assets\nosuchfile.jpg)

srv文件没有生成对应的头文件，原因是编译器版本不对，在`SmarterEyeSdk/wrapper/ros/src/zkhy_stereo_d/CMakeLists.txt`中配置一下c++11的编译选项即可。  

![c++](..\assets\c++.png)

- ### make 失败
![make](..\assets\make.png)

原因是编译时，没有找到Qt5.12。  
需要将安装的qt所依赖的库目录，添加到环境变量LD_LIBRARY_PATH下。  
eg：export LD_LIBRARY_PATH=/your/path/Qt5.12.2/5.12.2/gcc_64/lib:$LD_LIBRARY_PATH   