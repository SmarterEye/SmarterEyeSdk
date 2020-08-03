# ROS Wrapper 使用

## 安装

目前仅在`Ubuntu`系统下使用ROS Wrapper。ROS环境的注意事项可参考 [ROS环境](setup/env?id=ros)。

?> 这里分别列举在不同Ubuntu LTS版本下的安装命令。

* ROS Melodic (Ubuntu 18.04)

```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update
sudo apt install ros-melodic-desktop-full
sudo rosdep init
rosdep update
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

* ROS Kinetic (Ubuntu 16.04)

```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt-get update
sudo apt-get install ros-kinetic-desktop-full
sudo rosdep init
rosdep update
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrcs
```

## 使用ROS Wrapper

* 目录结构如下

```bash
<sdk>/wrapper
└── ros
    └── src
        └── zkhy_stereo_d
            ├── CMakeLists.txt
            ├── launch
            │   ├── display.launch
            │   └── zkhy_stereo.launch
            ├── package.xml
            ├── rviz
            │   └── zkhy_stereo.rviz
            ├── src
            │   ├── framemonitor.cpp
            │   ├── framemonitor.h
            │   ├── stereo_listener.cpp
            │   ├── stereo_publisher.cpp
            │   └── stereo_publisher.h
            └── srv
                ├── CameraParams.srv
                └── RotationMatrix.srv
```

!> 在编译ros wrapper前，需先对我们的SDK源码进行编译。相关步骤参考 [编译SDK源码](setup/build?id=编译SDK源码)。<br>

* 编译命令如下

```bash
cd <sdk>/wrapper/ros/
catkin_make
source devel/setup.bash
```

* 编译成功后，可执行 `roslaunch zkhy_stereo_d zkhy_stereo.launch` 来使用wrapper程序。
