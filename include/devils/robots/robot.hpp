#pragma once

namespace devils
{
    struct Robot
    {
        /**
         * Ran at the start of the Autonomous period.
         */
        virtual void autonomous() = 0;

        /**
         * Ran at the start of the Driver Control period.
         */
        virtual void opcontrol() = 0;

        /**
         * Ran at the start of the Disabled period.
         */
        virtual void disabled() = 0;
    };
}