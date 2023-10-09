#pragma once
#include "devils/chassis/tankChassis.h"
#include "pose.h"

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
        TankWheelOdometry(double wheelRadius, double wheelBase, double ticksPerRevolution);

        /**
         * Updates the odometry.
         * @param leftEncoder The left encoder ticks.
         * @param rightEncoder The right encoder ticks.
         * @param delta The time since the last update.
         */
        void update(int leftEncoder, int rightEncoder);

        /**
         * Updates the odometry.
         * @param chassis The chassis to update from.
         */
        void update(TankChassis *chassis);

        /**
         * Gets the current pose of the robot.
         */
        const Pose getPose();

        /**
         * Sets the current pose of the robot.
         * @param pose The pose to set the robot to.
         */
        void setPose(Pose pose);

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