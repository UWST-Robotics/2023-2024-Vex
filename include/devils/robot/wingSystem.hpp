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
         * @param wingsPort The ADI port of the wings
         */
        WingSystem(const uint8_t wingsPort)
            : pneumatics("WingsPneumatic", wingsPort)
        {
        }

        /**
         * Pops out the wings.
         */
        void extend()
        {
            pneumatics.extend();
            isExtended = true;
        }

        /**
         * Retracts the wings.
         */
        void retract()
        {
            pneumatics.retract();
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
        ScuffPneumatic pneumatics;
    };
}