#pragma once
#include "devils/robot/blaze.h"

devils::Blaze::Blaze() : chassis({FL_MOTOR_PORT, BL_MOTOR_PORT}, {FR_MOTOR_PORT, BR_MOTOR_PORT}),
                         odometry(WHEEL_RADIUS, WHEEL_BASE, TICKS_PER_REVOLUTION),
                         catapult(CATAPULT_MOTOR_PORT),
                         intake(INTAKE_MOTOR_PORT, MANIP_MOTOR_PORT),
                         wings(LEFT_WING_PORT, RIGHT_WING_PORT)
{
}

void devils::Blaze::updateOdometry()
{
    odometry.update(&chassis);
}