# 车道线数据

车道线数据头文件：`LdwDataInterface.h`

## 世界坐标系

1. 左镜头的中心作为世界坐标系的原点
2. 车辆前进方向为+Z轴
3. 平行路面且垂直Z轴的方向为X轴,右侧为+X
4. Y轴垂直路面, +Y指向路面
5. 如上建立坐标系后，路面平行于XZ平面

## 车道线边界

### 车道线边界方程

$$
x=C0 + C1 * z   
\tag{1}
$$

$$
x=C0 + C1 * z + C2 * z ^ 2                              
\tag{2}
$$

$$
x=C0 + C1 * z + C2 * z ^ 2 + C3 * z ^ 3
\tag{3}
$$

### 车道线边界结构体定义

```C++
struct LdwLaneBoundary
{
    int degree;
    float coefficient[4];
};
```

degree : 车道线边界的方程次数  
coefficient : 方程的系数。其中：  
`coefficient[0]` : C0  
`coefficient[1]` : C1  
`coefficient[2]` : C2  
`coefficient[3]` : C3  

## 车道线

```C++
struct LdwLane
{
    int width;						//单一车道线的宽度（单位：mm）
    int qualityGrade;				//车道线识别的质量（目前推荐通过类型判断车道线的质量）
    LdwLaneStyle style;				//车道线类型
    LdwLaneBoundary left_Boundary;  //车道线左侧边界
    LdwLaneBoundary rightBoundary;  //车道线右侧边界
};
```

### 车道线类型

```C++
enum LdwLaneStyle
{
    LDW_LANE_STYLE_NONE_LANE = 0,		//无车道线
    LDW_LANE_STYLE_PREDICT_LANE,		//预测的车道线
    LDW_LANE_STYLE_BROKEN_LANE,			//虚线
    LDW_LANE_STYLE_SOLID_LANE,			//实线
    LDW_LANE_STYLE_DOUBLE_BROKEN_LANE,	//双虚线
    LDW_LANE_STYLE_DOUBLE_SOLID_LANE,	//双实线
    LDW_LANE_STYLE_TRIPLE_LANE,			//其他
};
```

注意（以车辆所在车道为准）：  
内侧车道线是虚线的双车道线输出：LDW_LANE_STYLE_DOUBLE_BROKEN_LANE  
内侧车道线是实线的双车道线输出：LDW_LANE_STYLE_DOUBLE_SOLID_LANE  

## 路面

```C++
struct LdwRoadway
{
    int width[3];
    bool isTracking;
    LdwLane left_Lane;
    LdwLane rightLane;
    LdwLane adjacentLeft_Lane;
    LdwLane adjacentRightLane;
};
```

#### 解释说明：

width：两车道线间宽度，公式憋死条车道线，三个宽度，下标从左到右依次为0、1、2（mm）  
isTracking : 当前路面是否有车道线 (目前参数无效)  
left_Lane : 与坐标原点最近的左侧车道线  
rightLane : 与坐标原点最近的右侧车道线  
adjacentLeft_Lane :与左侧车道线最近的外侧车道线 (目前参数无效)  
adjacentRightLane : 与右侧车道线最近的外侧车道线 (目前参数无效)  
