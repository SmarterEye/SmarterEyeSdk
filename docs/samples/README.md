# 使用方法

!> *约束*：`<sdk>`表示的是我们提供编译好的SDK包在用户解压之后的绝对路径，或者`git clone`下来的SDK源码所在的路径，比如`/root/adas/Sdk-Release/` 或` /root/adas/SmarterEyeSdk`

我们提供下面一些Demo，是关于SDK的一些使用场景和用法。  
你可以在 `<sdk>/samples` 目录下找到工程样例。

```
samples
├── CMakeLists.txt
├── Disparity2RGBDemo
├── Disparity2Real3dDemo
├── DisplayFramesDemo
├── FirmwareUpdateDemo
├── GetCameraParamDemo
├── GetCompoundDemo
├── GetLaneExtDemo
├── GetMotionDataDemo
├── PointCloudDemo
├── StereoCameraDemo
├── common.cmake
├── find_opencv.cmake
└── find_pcl.cmake
```

这些Demo同样由`CMake`进行构建，下面介绍如何编译使用这些Demo。

## 准备

* 如果您使用的是*SDK源码*，编译`samples`工程之前，请确保 `<sdk>/src` 目录下的SDK源代码已经`make`通过，并且`install`完成。
* 如果您使用的是*打包编译好的SDK库*，请保证解压后的目录结构完整。

?> 由于`samples`同样基于`CMake`进行构建，所以在不通平台下，都可以用CMake来编译这些Demo。

## 编译

编译Samples与编译SDK源码过程类似。编译产生的Demo可执行程序，全都位于`<sdk>/bin`目录下面。

?> 如果您使用IDE来编写代码，可以使用支持CMake工程IDE（比如Qt Creator、CLion等），来打开samples目录下的CMake工程进行编译即可。  

### Linux平台

如需使用命令行进行编译，参考编译步骤如下：

```bash
cd SmarterEyeSdk
mkidr build && cd build
cmake ..
make -j
```

### Windows平台

这里列举两种方式进行编译。

* 在安装`Visual Studio 2017`后，打开 `适用于 VS 2017 的 x64 本机工具命令提示` 命令行界面，注意要是`x64`的才行。

```bash
cd samples
mkdir build
cd build
cmake .. -G "NMake Makefiles"
nmake
```

* 使用CMake生成`Visual Studio sln`工程

```bash
cd samples
mkdir build
cd build
cmake .. -G “Visual Studio 15 2017 Win64”
# 之后，双击打开build目录下的sln工程，使用Visual Studio 2017进行编译生成。
```

## 使用

?> 上述步骤只是完成的可执行程序的编译，若想成功执行这些Demo，需要把依赖库添加进`系统环境变量中`。

* Windows平台：把 `<sdk>/install/bin`的路径，添加进系统环境变量Path中
* Linux平台：  
`export LD_LIBRARY_PATH=<sdk>/bin:LD_LIBRARY_PATH`(使用的是我们提供编译好的SDK包)  
`export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH`(使用SDK源码进行编译并make install)
