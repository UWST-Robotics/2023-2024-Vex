#pragma once
#include "tankChassis.h"

devils::TankChassis::TankChassis(
    const std::vector<uint8_t> leftMotorPorts,
    const std::vector<uint8_t> rightMotorPorts) : leftMotors(leftMotorPorts),
                                                  rightMotors(rightMotorPorts)
{
}

void devils::TankChassis::move(double forward, double turn, double strafe = 0)
{
    leftMotors.move(MAX_VOLTAGE * (forward + turn));
    rightMotors.move(MAX_VOLTAGE * (forward - turn));
}

bool devils::TankChassis::isHolonomic()
{
    return false;
}

pros::Motor_Group *devils::TankChassis::getLeftMotors()
{
    return &leftMotors;
}

pros::Motor_Group *devils::TankChassis::getRightMotors()
{
    return &rightMotors;
}