#include "sedevicestate.h"

static const std::map<int, std::string> kErrorStringTable
{
    {0,     "No error"},
    {2,     "Camera: sensor error"},
    {4,     "Camera: internal watchdog timer error"},
    {5,     "Camera: PLDDR error"},
    {6,     "Camera: no remap data"},
    {7,     "Camera: ISP error"},
    {8,     "Camera: unknown error"},
    {11,    "Application: missing calibration file"},
    {19,    "Application: unknown error"},
    {21,    "VTDCP: device invalid"},
    {22,    "VTDCP: cable not connected"},
    {23,    "VTDCP: remote refuse"},
    {24,    "VTDCP: host not found"},
    {25,    "VTDCP: network resource error"},
    {26,    "VTDCP: connect timeout error"},
    {27,    "VTDCP: network error"},
    {28,    "VTDCP: unknown error"},
    {31,    "AutoInterface: can receive error"},
    {39,    "AutoInterface: unknown error"}
};

SEDeviceState::SEDeviceState()
{
    mDeviceState = SEDeviceState::NormalState;
    mErrorCodeList.clear();
}

SEDeviceState::SEDeviceState(const SEDeviceState &state_)
{
    mErrorCodeList.clear();
    for (uint8_t i =0; i < state_.getErrorCodeCount();i++) {
        mErrorCodeList.push_back(state_.errorCode(i));
    }
    mDeviceState = state_.currentState();
}

SEDeviceState::DeviceState SEDeviceState::currentState() const
{
    return mDeviceState;
}

uint8_t SEDeviceState::getErrorCodeCount() const
{
    return (uint8_t)mErrorCodeList.size();
}

int SEDeviceState::errorCode(uint8_t index) const{
    if(index <= 0 || index >= mErrorCodeList.size()){
        return mErrorCodeList[0];
    }else{
        return mErrorCodeList[index];
    }
}

std::string SEDeviceState::errormsg(int errCode) const{
    std::map<int, std::string>::const_iterator iter;
    iter = kErrorStringTable.find(errCode);
    std::string errmsg = "Error "+std::to_string(errCode)+": Unknown error";
    if(iter != kErrorStringTable.end()){
        errmsg = iter->second;
    }
    return errmsg;
}

void SEDeviceState::setDeviceState(SEDeviceState::DeviceState state)
{
    mDeviceState = state;
}

void SEDeviceState::setErrorCodeList(const int *list)
{
    mErrorCodeList.clear();
    for(int i=0;i<128;i++){
        if(list[i] == 0){
            break;
        }
        mErrorCodeList.push_back(list[i]);
    }
}

void SEDeviceState::clearErrorCodeList()
{
    mErrorCodeList.clear();
}
