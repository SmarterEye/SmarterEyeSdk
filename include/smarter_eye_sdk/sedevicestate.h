#ifndef DEVICESTATUSDEF_H
#define DEVICESTATUSDEF_H

#include "stereocameradef.h"
#include <vector>
#include <string>
#include <map>

class STEREO_SHARED_EXPORT SEDeviceState
{
public:
    enum DeviceState
    {
        NormalState,
        AbnormalState,
        HighTemperatureState
    };

    explicit SEDeviceState();
    SEDeviceState(const SEDeviceState &state_);
    DeviceState currentState() const;
    uint8_t getErrorCodeCount() const;
    int errorCode(uint8_t index = 0) const;
    std::string errormsg(int errCode) const;

    SEDeviceState &operator=(const SEDeviceState &state_){
        mErrorCodeList.clear();
        for (uint8_t i =0; i < state_.getErrorCodeCount();i++) {
            mErrorCodeList.push_back(state_.errorCode(i));
        }
        mDeviceState = state_.currentState();
        return *this;
    }

    void setDeviceState(DeviceState state);
    void setErrorCodeList(const int *list);
    void clearErrorCodeList();

private:

    DeviceState mDeviceState;
    std::vector<int> mErrorCodeList;
};


#endif // DEVICESTATUSDEF_H
