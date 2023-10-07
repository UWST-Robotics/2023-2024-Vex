#pragma once
#include "devils/robot/intakeSystem.h"

devils::IntakeSystem::IntakeSystem(uint8_t wheelPort, uint8_t manipPort)
    : wheelMotor(wheelPort),
      manipMotor(manipPort)
{
}

void devils::IntakeSystem::extend()
{
    manipMotor.move(MANIP_SPEED);
    isExtended = true;
}

void devils::IntakeSystem::retract()
{
    manipMotor.move(-MANIP_SPEED);
    isExtended = false;
}

void devils::IntakeSystem::intake()
{
    wheelMotor.move(WHEEL_SPEED);
    isIntaking = true;
    isOuttaking = false;
}

void devils::IntakeSystem::outtake()
{
    wheelMotor.move(-WHEEL_SPEED);
    isIntaking = false;
    isOuttaking = true;
}

void devils::IntakeSystem::stop()
{
    wheelMotor.move(0);
    isIntaking = false;
    isOuttaking = false;
}

const bool devils::IntakeSystem::getExtended()
{
    return isExtended;
}

const bool devils::IntakeSystem::getIntaking()
{
    return isIntaking;
}

const bool devils::IntakeSystem::getOuttaking()
{
    return isOuttaking;
}