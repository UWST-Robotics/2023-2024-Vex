#pragma once
#include "devils/robot/wingSystem.h"
#include <errno.h>
#include "devils/utils/logger.h"

devils::WingSystem::WingSystem(uint8_t leftWingPort, uint8_t rightWingPort)
    : leftWing(leftWingPort), rightWing(rightWingPort)
{
    if (errno != 0)
        Logger::error("WingSystem: ADI port is invalid");
}

void devils::WingSystem::extend()
{
    leftWing.set_value(true);
    rightWing.set_value(true);
    isExtended = true;
}

void devils::WingSystem::retract()
{
    leftWing.set_value(false);
    rightWing.set_value(false);
    isExtended = false;
}

const bool devils::WingSystem::getExtended()
{
    return isExtended;
}