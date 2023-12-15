#pragma once
#include "../hardware/scuffPneumatic.hpp"

namespace devils
{
    /**
     * Controls the pneumatic blocker that pops out on top of the robot.
     */
    class BlockerSystem
    {
    public:
        /**
         * Creates a new blocker system.
         * @param blockerPort The ADI port of the pneumatics
         */
        BlockerSystem(const uint8_t blockerPort)
            : pneumatics("BlockerPneumatic", blockerPort)
        {
        }

        /**
         * Pops out the blocker.
         */
        void extend()
        {
            pneumatics.extend();
            isExtended = true;
        }

        /**
         * Retracts the blocker.
         */
        void retract()
        {
            pneumatics.retract();
            isExtended = false;
        }

        /**
         * Gets if the blocker is extended.
         * @return True if the blocker is extended
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