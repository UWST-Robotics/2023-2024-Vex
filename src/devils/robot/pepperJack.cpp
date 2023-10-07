#pragma once
#include "devils/robot/pepperJack.h"

devils::PepperJack::PepperJack() : chassis({FL_MOTOR_PORT, BL_MOTOR_PORT}, {FR_MOTOR_PORT, BR_MOTOR_PORT}),
                                   odometry(WHEEL_RADIUS, WHEEL_BASE, TICKS_PER_REVOLUTION),
                                   intake(INTAKE_MOTOR_PORT, MANIP_MOTOR_PORT),
                                   wings(LEFT_WING_PORT, RIGHT_WING_PORT)
{
}

void devils::PepperJack::updateOdometry()
{
    odometry.update(&chassis);
}