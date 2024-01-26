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
         * Pops out the left wing.
         */
        void extendLeft()
        {
            leftPneumatic.extend();
        }

        /**
         * Pops out the right wing
         */
        void extendRight()
        {
            rightPneumatic.extend();
        }

        /**
         * Retracts the left wing
         */
        void retractLeft()
        {
            leftPneumatic.retract();
        }

        /**
         * Retracts the right wing
         */
        void retractRight()
        {
            rightPneumatic.retract();
        }

    private:
        ScuffPneumatic leftPneumatic;
        ScuffPneumatic rightPneumatic;
    };
}