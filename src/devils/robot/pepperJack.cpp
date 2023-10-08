#pragma once
#include "devils/robot/pepperJack.h"

devils::PepperJack::PepperJack() : chassis(L_MOTOR_PORTS, R_MOTOR_PORTS),
                                   odometry(WHEEL_RADIUS, WHEEL_BASE, TICKS_PER_REVOLUTION),
                                   intake(INTAKE_MOTOR_PORT, MANIP_MOTOR_PORT),
                                   wings(LEFT_WING_PORT, RIGHT_WING_PORT),
                                   climber(CLIMB_MOTOR_PORT)
{
}

void devils::PepperJack::updateOdometry()
{
    odometry.update(&chassis);
}