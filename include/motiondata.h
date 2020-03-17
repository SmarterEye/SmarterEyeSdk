#ifndef MOTIONDATA
#define MOTIONDATA

#pragma pack(push, 1)

struct MotionData
{
    double accelX;
    double accelY;
    double accelZ;
    double rotationX;
    double rotationY;
    double rotationZ;
    long long timestamp;
};

#pragma pack(pop)

#endif // MOTIONDATA

