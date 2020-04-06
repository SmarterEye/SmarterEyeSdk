# 环境依赖
不同平台下，SDK所要求的开发环境有所不同。

## 编译环境
* Windows平台下，建议使用的开发环境为 `vs2017 x86_64`。
* Linux平台下，建议使用`GNU`编译套件，要求编译器支持`C++ 11`及以上的标准。

## CMake
SDK源码基于CMake进行构建，请安装`CMake 3.0`及以上的版本。

## Qt
?> SDK所依赖的`Qt`版本为高于`5.10`。<br>
如果使用我们的SDK源码进行编译使用，需要自行安装符合要求的Qt版本，建议使用`Qt 5.12`; <br>
如果使用我们提供编译好的动态库包，SDK所依赖的Qt动态库已一同打包进动态库所在目录，无须另外安装即可使用。

## 附加（可选项）
以下依赖项**不影响**SDK源码的编译使用，只是部分示例Demo才需要用到。

### ROS
我们提供简单的ROS Wrapper，目前仅在Ubuntu系统下测试过。如需使用，请自行安装。  
安装ROS前，*请勿*用源码编译的安方式安装Qt、OpenCV、PCL、VTK等，否则可能导致ROS安装失败。
详细安装步骤请参考 http://wiki.ros.org/melodic/Installation/Ubuntu  
ROS有很多附加依赖，如果您在Ubuntu下，通过apt-get成功安装了ROS，则下面的`OpenCV`、`PCL`就不用再次安装了。

!> ROS通过apt-get安装时，会一并安装`qt-default`。这里安装的Qt版本比较旧，*不符合*SDK源码依赖的Qt最低版本`Qt5.10`，如果您是源码编译SDK，请从[Qt官网](https://download.qt.io/archive/qt/) 另外安装更高版本的Qt，并在cmake文件中，指定这个Qt库目录。<br>
注意，从官网自行下载安装的Qt目录，不要加到`PATH`或`LD_LIBRARY_PATH`这些环境变量里面，否则会和apt-get安装的qt-default冲突，使ROS部分功能无法正常运行。<br>

### OpenCV
部分Demo使用到了OpenCV的模块，如需使用OpenCV，请参考对应的安装文档。  
* Windows平台：https://docs.opencv.org/master/d3/d52/tutorial_windows_install.html
* Linux平台：https://docs.opencv.org/master/d7/d9f/tutorial_linux_install.html

?> Ubuntu下，推荐使用apt-get安装OpenCV：`sudo apt-get install libopencv-dev`  

### PCL
部分Demo使用了`点云`数据，如需使用点云相关的功能，请安装`PCL`库。

* Windows平台：从 https://github.com/PointCloudLibrary/pcl/releases 下载集成安装程序(PCL + dependencies)
* Linux平台：参考官方文档，[PCL Installation](http://www.pointclouds.org/documentation/tutorials/compiling_pcl_posix.php)。