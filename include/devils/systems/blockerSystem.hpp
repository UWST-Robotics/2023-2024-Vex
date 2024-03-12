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
         * Forces the robot to climb after the elapsed time.
         */
        void autoClimb()
        {
            int elapsedTime = pros::millis() - startTime;
            if (elapsedTime > AUTO_CLIMB_TIME)
                retract(true);
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
         * @param climb Whether or not the robot should climb.
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
         * Restarts the climb timer
         */
        void restart()
        {
            startTime = pros::millis();
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
        static constexpr int AUTO_CLIMB_TIME = 1000 * 119; // 1:59

        bool isExtended = false;
        int startTime = pros::millis();
        ScuffPneumatic downPneumatics;
        ScuffPneumatic upPneumatics;
    };
}