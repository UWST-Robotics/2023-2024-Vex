#pragma once
#include "devils/robot/catapultSystem.h"
#include <errno.h>
#include "devils/utils/logger.h"

devils::CatapultSystem::CatapultSystem(int8_t motorPort)
    : catapultMotor(motorPort)
{
    if (errno != 0)
        Logger::error("IntakeSystem: motor port is invalid");
}

void devils::CatapultSystem::fire()
{
    catapultMotor.move(MOTOR_SPEED);
    isFiring = true;
}

void devils::CatapultSystem::stop()
{
    catapultMotor.move(0);
    isFiring = false;
}

const bool devils::CatapultSystem::getFiring()
{
    return isFiring;
}