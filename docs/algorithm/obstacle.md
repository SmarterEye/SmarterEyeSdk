# 障碍物数据

障碍物数据头文件：`obstacleData.h`

## 障碍物提取功能描述

1. 支持3~30米范围内的障碍物提取和测距
2. 检测并跟踪本车道内的障碍物
3. 支持对最近的一个障碍物的提取

## 对外传输的障碍物信息描述说明

```C++
#ifndef _OBSTACLE_DATA_H_
#define _OBSTACLE_DATA_H_

struct OutputObstacles
{
    float currentSpeed;                 //(m/second) 当前帧自车速度
    float frameRate;                    //(fps) 图像帧率

    unsigned char trackId;              //障碍物跟踪ID
    unsigned char trackFrameNum;        //障碍物跟踪帧数，每帧递增，直至增加到最大值
    unsigned char stateLabel;           //前向碰撞障碍物 0：无效障碍物或连续型障碍物 1：报警区域内的最近障碍物，2：报警区域内的障碍物，3：报警区域外障碍物

    unsigned char classLabel;           //障碍物分类标签 0：无效 1：正在报警的障碍物 2：需要报警的障碍物 3：不报警障碍物 4：左侧连续型障碍物 5:右侧连续型障碍物 6:估计的已消失障碍物
    unsigned char continuousLabel;      //连续型障碍物分类标签：0：无效障碍物 ，1：左侧连续型障碍物 2：人 3：右侧连续型障碍物
    unsigned char fuzzyEstimationValid; //(0/1) 0: current fuzzy estimation is invalid; 1: current fuzzy estimation is valid
    RecognitionType obstacleType;       //障碍物类型

    float avgDisp;                      //(pixel) 障碍物平均视差
    float avgDistanceZ;                 //(m) Z方向上障碍物的平均距离
    float nearDistanceZ;                //(m) 连续型障碍物在Z方向上的最近距离
    float farDistanceZ;                 //(m) 连续型障碍物在Z方向上的最远距离

    float real3DLeftX;                  //(-+m) 障碍物左侧到原点的横向距离（原点X在车辆中心，且向右为正）
    float real3DRightX;                 //(-+m) 障碍物右侧到原点的横向距离（原点X在车辆中心，且向右为正）
    float real3DCenterX;                //(-+m) 障碍物中心到原点的横向距离（原点X在车辆中心，且向右为正）
    float real3DUpY;                    //(-+m) 障碍物上侧到原点的竖向距离（原点Y在车辆中心，且向下为正）
    float real3DLowY;                   //(-+m) 障碍物下侧到原点的竖向距离（原点Y在车辆中心，且向下为正）

    unsigned short firstPointX;         //(pixel) 图像坐标下矩形框的左上点的坐标X
    unsigned short firstPointY;         //(pixel) 图像坐标下矩形框的左上点的坐标Y
    unsigned short secondPointX;        //(pixel) 图像坐标下矩形框的右上点的坐标X
    unsigned short secondPointY;        //(pixel) 图像坐标下矩形框的右上点的坐标Y
    unsigned short thirdPointX;         //(pixel) 图像坐标下矩形框的右下点的坐标X
    unsigned short thirdPointY;         //(pixel) 图像坐标下矩形框的右下点的坐标Y
    unsigned short fourthPointX;        //(pixel) 图像坐标下矩形框的左下点的坐标X
    unsigned short fourthPointY;        //(pixel) 图像坐标下矩形框的左下点的坐标Y

    float fuzzyRelativeDistanceZ;       //(m) Z方向上相对估计距离
    float fuzzyRelativeSpeedZ;          //(m/second) Z方向上障碍物的相对估计速度
    float fuzzyCollisionTimeZ;          //(second) Z方向上估计的碰撞时间

    unsigned char fuzzyCollisionX;      //(0/1) 估计X方向上是否会发生碰撞
    float fuzzy3DWidth;                 //(m) 预估障碍物的宽度
    float fuzzy3DCenterX;               //(-+m) 预估的障碍物中心到原点的横向距离（原点X在车辆中心，且向右为正）
    float fuzzy3DLeftX;                 //(-+m) 预估的障碍物左侧到原点的横向距离（世界坐标下的原点X在车辆中心，且向右为正）
    float fuzzy3DRightX;                //(-+m) 预估的障碍物右侧到原点的横向距离（世界坐标下的原点X在车辆中心，且向右为正）
    float fuzzy3DHeight;                //(m) 预估的障碍物高度
    float fuzzy3DUpY;                   //(-+m) 预估的障碍物上侧到原点的竖向距离（原点Y在车辆中心，且向上为正）
    float fuzzy3DLowY;                  //(-+m) 预估的障碍物下侧坐标Y到原点的竖向距离（原点Y在车辆中心，且向上为正）

    float fuzzyRelativeSpeedCenterX;    //(m/second) 当前障碍物中心在X方向上的估计速度
    float fuzzyRelativeSpeedLeftX;      //(m/second) 当前障碍物左边界在X方向上的估计速度
    float fuzzyRelativeSpeedRightX;     //(m/second) 当前障碍物右边界在X方向上的估计速度

#ifdef EXTEND_INFO_ENABLE
    unsigned char storeId;              //current obstacle store id in obstacle detection buffer
#endif //EXTEND_INFO_ENABLE
};
#endif // _OBSTACLE_DATA_H_
```

障碍物类型的定义如下：

```C++
enum RecognitionType
{
    INVALID = 0,
    VEHICLE,
    PEDESTRIAN,
    CHILD,
    BICYCLE,
    MOTO,
    TRUCK,
    BUS,
    OTHERS,
    ESTIMATED,
    CONTINUOUS
};
```

## 需要外部输入的信息说明

1. 相机的安装高度；（单位：m）
2. 车辆左侧距离左相机的距离；（单位：m）
3. 车辆右侧距离左相机的距离；（单位：m）
4. 车头最前部距离左相机的纵向距离；（单位：m）
5. 车辆的宽度；（单位：m）
6. 车头的高度；（单位：m）
