#pragma once

namespace devils
{
    struct Robot
    {
        /**
         * Ran at the start of the Autonomous period.
         */
        void autonomous() {}

        /**
         * Ran at the start of the Driver Control period.
         */
        void opcontrol() {}
    };
}