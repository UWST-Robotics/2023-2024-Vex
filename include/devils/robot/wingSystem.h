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
        WingSystem(uint8_t leftWingPort, uint8_t rightWingPort);

        /**
         * Pops out the wings.
         */
        void extend();

        /**
         * Retracts the wings.
         */
        void retract();

        /**
         * Returns true if the wings are extended.
         */
        const bool getExtended();

    private:
        bool isExtended = false;
        pros::ADIDigitalOut leftWing;
        pros::ADIDigitalOut rightWing;
    };
}