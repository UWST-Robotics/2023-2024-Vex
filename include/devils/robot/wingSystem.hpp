#pragma once
#include "../hardware/scuffPneumatic.hpp"

namespace devils
{
    /**
     * Controls the pneumatic wings that pop out to the sides of the robot.
     */
    class WingSystem
    {
    public:
        /**
         * Creates a new wing system.
         * @param leftPort The ADI port of the left wing pneumatic.
         * @param rightPort The ADI port of the right wing pneumatic.
         */
        WingSystem(const uint8_t leftPort, const uint8_t rightPort)
            : leftPneumatic("WingsLeftPneumatic", leftPort),
              rightPneumatic("WingsRightPneumatic", rightPort)
        {
        }

        /**
         * Pops out the wings.
         */
        void extend()
        {
            leftPneumatic.extend();
            rightPneumatic.extend();
            isExtended = true;
        }

        /**
         * Retracts the wings.
         */
        void retract()
        {
            leftPneumatic.retract();
            rightPneumatic.retract();
            isExtended = false;
        }

        /**
         * Gets if the wings are extended.
         * @return True if the wings are extended
         */
        const bool getExtended()
        {
            return isExtended;
        }

    private:
        bool isExtended = false;
        ScuffPneumatic leftPneumatic;
        ScuffPneumatic rightPneumatic;
    };
}