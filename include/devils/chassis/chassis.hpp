#pragma once
#include <algorithm>

namespace devils
{
    /**
     * Represents a base chassis. A chassis is a robot's drivetrain.
     */
    struct BaseChassis
    {
        /**
         * Moves the robot in a direction using voltage.
         * @param forward The forward speed of the robot from -1 to 1.
         * @param turn The turn speed of the robot from -1 to 1.
         * @param strafe The strafe speed of the robot from -1 to 1.
         */
        virtual void move(double forward, double turn, double strafe = 0) = 0;

        /**
         * Stops the robot.
         */
        virtual void stop()
        {
            move(0, 0, 0);
        };

        /**
         * Sets the speed of the robot.
         * @param speed The speed of the robot from 0 to 1.
         */
        virtual void setSpeed(double speed)
        {
            this->speed = std::clamp(speed, 0.0, 1.0);
        }

        /**
         * Gets the speed of the robot.
         * @return The speed of the robot from -1 to 1.
         */
        virtual double getSpeed()
        {
            return speed;
        }

    protected:
        double speed = 1.0;
    };
}