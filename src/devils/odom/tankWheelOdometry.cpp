#pragma once
#include "devils/odom/tankWheelOdometry.h"
#include "pros/rtos.hpp"
#include <cmath>

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
    double deltaX = deltaDistance * std::cos(rotation + deltaRotation / 2);
    double deltaY = deltaDistance * std::sin(rotation + deltaRotation / 2);

    // Update X, Y, and Rotation
    x += deltaX;
    y += deltaY;
    rotation += deltaRotation;
}

void devils::TankWheelOdometry::update(devils::TankChassis *chassis)
{
    update(
        chassis->getLeftMotors()->get_positions()[0],
        chassis->getRightMotors()->get_positions()[0]);
}

double devils::TankWheelOdometry::getX()
{
    return x;
}

double devils::TankWheelOdometry::getY()
{
    return y;
}

double devils::TankWheelOdometry::getRotation()
{
    return rotation;
}

void devils::TankWheelOdometry::setX(double x)
{
    this->x = x;
}

void devils::TankWheelOdometry::setY(double y)
{
    this->y = y;
}

void devils::TankWheelOdometry::setRotation(double rotation)
{
    this->rotation = rotation;
}