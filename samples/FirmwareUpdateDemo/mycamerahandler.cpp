#include <iostream>
#include <string>

#include "mycamerahandler.h"
#include "satpext.h"

MyCameraHandler::MyCameraHandler(std::string name):
    mName(name)
{}

void MyCameraHandler::handleUpdateFinished(Result result)
{
    mUpgradeResult = result;
    if (!mUpgradeResult.successed) {
        std::cout << "Update failed with warning:"<< mUpgradeResult.warning << std::endl;
        std::cout << "If you want to update firmware, please rerun the demo to try again !!! " << std::endl;
    } else {
        std::cout << "Update succeed!" << std::endl;
    }
}
