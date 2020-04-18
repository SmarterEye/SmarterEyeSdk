#ifndef MOTIONDATAHANDLER_H
#define MOTIONDATAHANDLER_H

struct MotionData;

class MotionDataHandler
{
public:
    virtual void handleMotionData(const MotionData *motionData) = 0;
};

#endif // MOTIONDATAHANDLER_H
