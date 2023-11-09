#pragma once
#include "../chassis/tankChassis.hpp"
#include "pose.hpp"
#include "pros/rtos.hpp"
#include "devils/utils/logger.hpp"
#include <cmath>
#include <errno.h>

#define M_PI 3.14159265358979323846

namespace devils
{
    class TankWheelOdometry
    {
    public:
        /**
         * Represents a tank wheel odometry system.
         * Position is calculated using the left and right encoder values.
         * @param wheelRadius The radius of the wheels in inches.
         * @param wheelBase The distance between the wheels in inches.
         * @param ticksPerRevolution The number of ticks per revolution of the encoders.
         */
        TankWheelOdometry(double wheelRadius, double wheelBase, double ticksPerRevolution)
        {
            this->wheelRadius = wheelRadius;
            this->wheelBase = wheelBase;
            this->ticksPerRevolution = ticksPerRevolution;
            lastUpdateTimestamp = pros::millis();
        }

        /**
         * Updates the odometry.
         * @param leftEncoder The left encoder ticks.
         * @param rightEncoder The right encoder ticks.
         * @param delta The time since the last update.
         */
        void update(int leftEncoder, int rightEncoder)
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

        /**
         * Updates the odometry.
         * @param chassis The chassis to update from.
         */
        void update(TankChassis *chassis)
        {
            update(
                chassis->getLeftMotors()->get_positions()[0],
                chassis->getRightMotors()->get_positions()[0]);
            if (errno != 0)
                Logger::error("TankWheelOdometry: Failed to update from chassis");
        }

        /**
         * Gets the current pose of the robot.
         */
        const Pose getPose()
        {
            return currentPose;
        }

        /**
         * Sets the current pose of the robot.
         * @param pose The pose to set the robot to.
         */
        void setPose(Pose pose)
        {
            currentPose = pose;
        }

    private:
        double wheelRadius;
        double wheelBase;
        double ticksPerRevolution;

        Pose currentPose;

        uint32_t lastUpdateTimestamp = 0;
        double lastLeft = 0;
        double lastRight = 0;
    };
}