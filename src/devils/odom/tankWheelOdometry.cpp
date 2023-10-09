#pragma once
#include "devils/odom/tankWheelOdometry.h"
#include "pros/rtos.hpp"
#include <cmath>
#include "devils/utils/logger.h"
#include <errno.h>

#define M_PI 3.14159265358979323846

devils::TankWheelOdometry::TankWheelOdometry(double wheelRadius, double wheelBase, double ticksPerRevolution)
{
    this->wheelRadius = wheelRadius;
    this->wheelBase = wheelBase;
    this->ticksPerRevolution = ticksPerRevolution;
    lastUpdateTimestamp = pros::millis();
}

void devils::TankWheelOdometry::update(int leftEncoder, int rightEncoder)
{
    // Get Delta Time
    uint32_t deltaT = lastUpdateTimestamp - pros::millis();
    lastUpdateTimestamp = pros::millis();

    // Get Distance
    double left = leftEncoder / ticksPerRevolution * 2 * M_PI * wheelRadius;
    double right = rightEncoder / ticksPerRevolution * 2 * M_PI * wheelRadius;

    // Get Delta Distance
    double deltaLeft = left - lastLeft;
    double deltaRight = right - lastRight;
    lastLeft = left;
    lastRight = right;

    // Calculate Delta Distance
    double deltaDistance = (deltaLeft + deltaRight) / 2;
    double deltaRotation = (deltaRight - deltaLeft) / wheelBase;

    // Calculate Delta X and Y
    double deltaX = deltaDistance * std::cos(currentPose.rotation + deltaRotation / 2);
    double deltaY = deltaDistance * std::sin(currentPose.rotation + deltaRotation / 2);

    // Update X, Y, and Rotation
    currentPose.x += deltaX;
    currentPose.y += deltaY;
    currentPose.rotation += deltaRotation;
}

void devils::TankWheelOdometry::update(devils::TankChassis *chassis)
{
    update(
        chassis->getLeftMotors()->get_positions()[0],
        chassis->getRightMotors()->get_positions()[0]);
    if (errno != 0)
        Logger::error("TankWheelOdometry: Failed to update from chassis");
}

const devils::Pose devils::TankWheelOdometry::getPose()
{
    return currentPose;
}

void devils::TankWheelOdometry::setPose(Pose pose)
{
    currentPose = pose;
}