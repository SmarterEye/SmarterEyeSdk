# 双目相机标定参数

标定参数头文件：`calibrationParams.h`

## 标定参数获取功能描述

支持获取相机标定参数（双目及单目）。

## 标定参数说明

### 立体相机内参及外参

以下参数对应立体相机 remap校正后的左右图像

```C++
struct StereoCalibrationParameters
{
    double focus;                       //(pixel) 以像素为单位的相机焦距，此参数左右相机相同

    double cx;                          //(pixel) 图像中光心的x坐标
    double cy;                          //(pixel) 图像中光心的y坐标

    double RRoll;                       //(rad) 角度向量：旋转角，俯仰角，偏航角
    double RPitch;
    double RYaw;

    double Tx;                          //(mm) 旋转矩阵
    double Ty;
    double Tz;
};
```

### 单目相机标定参数

```C++
struct MonoCalibrationParameters
{
    double fx;                          //(pixel) 以像素为单位的相机x方向的焦距 pixel.
    double fy;                          //(pixel) 以像素为单位的相机y方向的焦距

    double cx;                          //(pixel) 图像中光心的x坐标
    double cy;                          //(pixel) 图像中光心的y坐标

    double k1;                          //径向畸变参数
    double k2;
    double k3;

    double p1;                          //切向畸变参数
    double p2;
};
```

***

## 旋转矩阵

旋转矩阵头文件：`rotationmatrix.h`

### 旋转矩阵获取功能描述

支持获取相机学习结果中用于图像坐标系与世界坐标系之间转换的两个旋转矩阵。
运行Demo连接设备成功后，输入 “r” + 回车键，获取到的旋转矩阵会保存在本地 `/home/rotation_matrix.txt` 文件中。
在完成相机学习，生成相机学习结果后，需重启设备，以加载正确旋转矩阵参数。

### 旋转矩阵说明

```C++
static const int kNumReal3DToImage = 12;
static const int kNumImageToReal3D = 9;

struct RotationMatrix
{
    float real3DToImage[kNumReal3DToImage];
    float imageToReal3D[kNumImageToReal3D];
};
```

#### 使用公式

`real3DToImage[12]`：3×4的矩阵M，用于将世界坐标系中坐标（X<sub>w</sub>,Y<sub>w</sub>,Z<sub>w</sub>）转换为图像坐标系坐标（x，y）。

$$
\begin{bmatrix} tempX \\ tempY \\ Z \\ \end{bmatrix} = \begin{bmatrix} m_0&m_1&m_2&m_3 \\ m_4&m_5&m_6&m_7 \\ m_8&m_9&m_{10}&m_{11} \\ \end{bmatrix} \begin{bmatrix} X_w \\ Y_w \\ Z_w \\ 1 \\ \end{bmatrix}
$$

$$
\begin{bmatrix} x \\ y \\ 1 \\ \end{bmatrix} = \begin{bmatrix} tempX \\ tempY \\ Z \\ \end{bmatrix} / Z
$$

`imageToReal3D[9]`：3×3的矩阵M，用于将图像坐标系中坐标（x，y）转换为世界坐标系中坐标（X<sub>w</sub>,Y<sub>w</sub>,Z<sub>w</sub>）。
$$
\begin{bmatrix} tempX \\ tempY \\ tempZ \\ \end{bmatrix} = \begin{bmatrix} m_0&m_1&m_2 \\ m_3&m_4&m_5 \\ m_6&m_7&m_8 \\ \end{bmatrix} \begin{bmatrix} x \\ y \\ 1 \\ \end{bmatrix}
$$
$$
\frac{X_w}{tempX} = \frac{Y_w}{tempY} = \frac{Z_w}{tempZ} = k
$$

根据X<sub>w</sub>,Y<sub>w</sub>,Z<sub>w</sub>任意一个，求得比例参数k，进而求得坐标（X<sub>w</sub>,Y<sub>w</sub>,Z<sub>w</sub>）。
