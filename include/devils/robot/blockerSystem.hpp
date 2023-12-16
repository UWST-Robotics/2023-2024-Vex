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
         * @param downPneumaticPort The ADI port of the down blocker pneumatic.
         * @param upPneumaticPort The ADI port of the up blocker pneumatic.
         */
        BlockerSystem(const uint8_t downPneumaticPort, const uint8_t upPneumaticPort)
            : downPneumatics("BlockerDownPneumatic", downPneumaticPort),
              upPneumatics("BlockerUpPneumatic", upPneumaticPort)
        {
        }

        /**
         * Pops out the blocker.
         */
        void extend()
        {
            upPneumatics.extend();
            downPneumatics.retract();
            isExtended = true;
        }

        /**
         * Retracts the blocker.
         */
        void retract(bool climb = false)
        {
            upPneumatics.retract();
            if (climb)
                downPneumatics.extend();
            else
                downPneumatics.retract();
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
        ScuffPneumatic downPneumatics;
        ScuffPneumatic upPneumatics;
    };
}