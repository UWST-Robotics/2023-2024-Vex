#pragma once
#include "devils/robot/climbSystem.h"
#include <errno.h>
#include "devils/utils/logger.h"

devils::ClimbSystem::ClimbSystem(uint8_t motorPort)
    : climbMotor(motorPort)
{
    if (errno != 0)
        Logger::error("ClimbSystem: motor port is invalid");
}

void devils::ClimbSystem::climb()
{
    climbMotor.move(MOTOR_SPEED);
    isClimbing = true;
    isDropping = false;
}

void devils::ClimbSystem::stop()
{
    climbMotor.move(0);
    isClimbing = false;
    isDropping = false;
}

void devils::ClimbSystem::drop()
{
    climbMotor.move(-MOTOR_SPEED);
    isClimbing = false;
    isDropping = true;
}

const bool devils::ClimbSystem::getClimbing()
{
    return isClimbing;
}

const bool devils::ClimbSystem::getDropping()
{
    return isDropping;
}