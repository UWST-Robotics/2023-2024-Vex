#pragma once
#include "devils/robot/catapultSystem.h"

devils::CatapultSystem::CatapultSystem(uint8_t motorPort)
    : catapultMotor(motorPort)
{
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