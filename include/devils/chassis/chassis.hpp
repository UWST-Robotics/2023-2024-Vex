#pragma once

namespace devils
{
    struct BaseChassis
    {
    public:
        /**
         * Moves the robot in a direction using voltage.
         * @param forward The forward speed of the robot from -1 to 1.
         * @param turn The turn speed of the robot from -1 to 1.
         * @param strafe The strafe speed of the robot from -1 to 1.
         */
        virtual void move(double forward, double turn, double strafe = 0) {}

        /**
         * Returns whether or not the chassis is holonomic.
         * @return true if the chassis is holonomic, false otherwise.
         */
        virtual bool isHolonomic()
        {
            return false;
        }
    };
}