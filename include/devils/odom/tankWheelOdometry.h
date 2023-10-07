#pragma once
#include "devils/chassis/tankChassis.h"

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
         * Gets the x position of the robot.
         * @return The x position of the robot in inches.
         */
        double getX();

        /**
         * Gets the y position of the robot.
         * @return The y position of the robot in inches.
         */
        double getY();

        /**
         * Gets the rotation of the robot.
         * @return The rotation of the robot in radians.
         */
        double getRotation();

        /**
         * Sets the x position of the robot.
         * @param x The x position of the robot in inches.
         */
        void setX(double x);

        /**
         * Sets the y position of the robot.
         * @param y The y position of the robot in inches.
         */
        void setY(double y);

        /**
         * Sets the rotation of the robot.
         * @param rotation The rotation of the robot in radians.
         */
        void setRotation(double rotation);

    private:
        double wheelRadius;
        double wheelBase;
        double ticksPerRevolution;

        double x = 0;
        double y = 0;
        double rotation = 0;

        uint32_t lastUpdateTimestamp = 0;
        double lastLeft = 0;
        double lastRight = 0;
    };
}