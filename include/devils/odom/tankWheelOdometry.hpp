#pragma once
#include "../chassis/tankChassis.hpp"
#include "pose.hpp"
#include "pros/rtos.hpp"
#include "devils/utils/logger.hpp"
#include "odomSource.hpp"
#include <cmath>
#include <errno.h>

#define M_PI 3.14159265358979323846

namespace devils
{
    class TankWheelOdometry : public OdomSource
    {
    public:
        /**
         * Represents a tank wheel odometry system.
         * Position is calculated using the left and right encoder values.
         * @param wheelRadius The radius of the wheels in inches.
         * @param wheelBase The distance between the wheels in inches.
         * @param ticksPerRevolution The number of ticks per revolution of the encoders.
         */
        TankWheelOdometry(const double wheelRadius, const double wheelBase, const double ticksPerRevolution)
            : wheelRadius(wheelRadius),
              wheelBase(wheelBase),
              ticksPerRevolution(ticksPerRevolution)
        {
            this->currentPose.x = 0;
            this->currentPose.y = 0;
            this->currentPose.rotation = 0;
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
            double left = (leftEncoder / ticksPerRevolution) * 2 * M_PI * wheelRadius;
            double right = (rightEncoder / ticksPerRevolution) * 2 * M_PI * wheelRadius;

            // Get Delta Distance
            double deltaLeft = left - lastLeft;
            double deltaRight = right - lastRight;
            lastLeft = left;
            lastRight = right;

            // Calculate Delta Distance
            double deltaDistance = (deltaLeft + deltaRight) / 2;
            double deltaRotation = (deltaLeft - deltaRight) / wheelBase;

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
            auto leftPositions = chassis->getLeftMotors()->get_positions();
            auto rightPositions = chassis->getRightMotors()->get_positions();

            // Check Position State
            if (leftPositions.size() < 1)
            {
                Logger::warn("TankWheelOdometry: Failed to update from left positions");
                return;
            }
            if (rightPositions.size() < 1)
            {
                Logger::warn("TankWheelOdometry: Failed to update from right positions");
                return;
            }

            // Update
            update(leftPositions[0], rightPositions[0]);
        }

        /**
         * Gets the current pose of the robot.
         */
        const Pose getPose() override
        {
            return currentPose;
        }

        /**
         * Sets the current pose of the robot.
         * @param pose The pose to set the robot to.
         */
        void setPose(Pose pose) override
        {
            currentPose = pose;
        }

    private:
        const double wheelRadius;
        const double wheelBase;
        const double ticksPerRevolution;

        Pose currentPose;

        uint32_t lastUpdateTimestamp = 0;
        double lastLeft = 0;
        double lastRight = 0;
    };
}