# ROS Wrapper 说明

执行完上一步的wrapper launch程序后，有下面这些 `topic` 以及 `service` 产生。

`rostopic list`
```bash
/zkhy_stereo/disparity      # 深度图数据

/zkhy_stereo/left/color     # 左相机彩色图
/zkhy_stereo/left/gray      # 左相机灰度图

/zkhy_stereo/right/color    # 右相机彩色图
/zkhy_stereo/right/gray     # 右相机灰度图
```

`rosservice list`
```bash
/zkhy_stereo/get_camera_params      # 获取相机参数
/zkhy_stereo/get_rotation_matrix    # 获取旋转矩阵数据
```
