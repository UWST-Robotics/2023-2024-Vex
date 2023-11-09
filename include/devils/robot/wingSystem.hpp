#pragma once
#include "pros/adi.hpp"

namespace devils
{
    class WingSystem
    {
    public:
        /**
         * Controls the pneumatic wings that pop out to the sides of the robot.
         */
        WingSystem(uint8_t leftWingPort, uint8_t rightWingPort)
            : leftWing(leftWingPort),
              rightWing(rightWingPort)
        {
            if (errno != 0)
                Logger::error("WingSystem: ADI port is invalid");
        }

        /**
         * Pops out the wings.
         */
        void extend()
        {
            leftWing.set_value(true);
            rightWing.set_value(true);
            isExtended = true;
        }

        /**
         * Retracts the wings.
         */
        void retract()
        {
            leftWing.set_value(false);
            rightWing.set_value(false);
            isExtended = false;
        }

        /**
         * Returns true if the wings are extended.
         */
        const bool getExtended()
        {
            return isExtended;
        }

    private:
        bool isExtended = false;
        pros::ADIDigitalOut leftWing;
        pros::ADIDigitalOut rightWing;
    };
}