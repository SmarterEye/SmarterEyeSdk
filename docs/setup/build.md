# 编译SDK源码

?> 该步骤是介绍如何使用SDK源码进行编译，如果您使用的是我们编译好的SDK动态库，请跳过这一步。

## 下载SDK源码
SDK源码位于GitHub仓库，地址为 https://github.com/loadxtp/SmarterEyeSdk
```
git clone https://github.com/loadxtp/SmarterEyeSdk.git

目录结构
SmarterEyeSdk
├── 3rdparty
├── CMakeLists.txt
├── LICENSE
├── README.md
├── cmake
├── docs
├── include
├── samples
├── smarter_eye_sdk-config.cmake.in
├── src
└── wrapper
```

## 编译
SDK使用`CMake`进行构建。  
由于我们的SDK依赖Qt，**推荐**在安装Qt后，使用`Qt Creator`打开此工程。

### Linux平台

如需使用命令行进行编译，参考编译步骤如下：

```bash
cd SmarterEyeSdk
mkidr build && cd build
cmake ..
make -j
sudo make install
```

?> 执行`make install`之后，会把编译完成的`smarter_eye_sdk`动态库，安装到`/usr/local/lib`下

### Windows平台

这里列举两种方式进行编译。

* 在安装`Visual Studio 2017`后，打开 `适用于 VS 2017 的 x64 本机工具命令提示` 命令行界面，注意要是`x64`的才行。

```bash
cd SmarterEyeSdk
mkdir build
cd build
cmake .. -G "NMake Makefiles"
nmake
nmake install
```

?> 执行`nmake install`之后，会把编译完成的`smarter_eye_sdk`动态库，安装到`<sdk>/install`下。


* 使用CMake生成`Visual Studio sln`工程

```bash
cd SmarterEyeSdk
mkdir build
cd build
cmake .. -G “Visual Studio 15 2017 Win64”
# 之后，双击打开build目录下的sln工程，使用Visual Studio 2017进行编译生成。
```

## 使用方式

### 使用方式举例

* 这里介绍如何使用 `CMake` 创建的项目来使用 `SDK` 。

```
如果使用SDK源码进行编译，install完成后，可以使用find_package来找到刚安装的sdk库
if(WIN32)
    set(CMAKE_PREFIX_PATH ${CMAKE_CURRENT_LIST_DIR}/../install)
endif()

find_package(smarter_eye_sdk)

if(smarter_eye_sdk_FOUND)

    message(STATUS "use installed smarter_eye_sdk from ${smarter_eye_sdk_LIBRARY_DIRS}")
    message(STATUS ${smarter_eye_sdk_INCLUDE_DIRS})
    message(STATUS ${smarter_eye_sdk_LIBRARY_DIRS})
    message(STATUS ${smarter_eye_sdk_LIBRARIES})

    include_directories(${smarter_eye_sdk_INCLUDE_DIRS})
    link_directories(${smarter_eye_sdk_LIBRARY_DIRS})
    link_libraries(${smarter_eye_sdk_LIBRARIES})

else()

    # 如果使用的是我们提供编译好的SDK动态库包，则不需要用find_package，直接使用相对路径的相关库就行了

    message(STATUS "try to use released binary libs")

    set(SDK_RELEASE_ROOT "${CMAKE_CURRENT_LIST_DIR}/..")
    set(SDK_INCLUDE_DIR ${SDK_RELEASE_ROOT}/include/smarter_eye_sdk)
    set(SDK_BIN_DIR ${SDK_RELEASE_ROOT}/bin)
    set(SDK_LIB_DIR ${SDK_BIN_DIR})
    set(SDK_LIBS StereoCamera ImageUtils)

    include_directories(${SDK_INCLUDE_DIR})
    link_directories(${SDK_BIN_DIR})
    link_libraries(${SDK_LIBS})

endif()
```
