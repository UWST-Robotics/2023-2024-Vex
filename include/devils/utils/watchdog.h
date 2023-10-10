#pragma once
#include <functional>

namespace devils
{
    class Watchdog
    {
    public:
        /**
         * Adds a check to the watchdog.
         * @param check A function that returns true if the robot is unsafe.
         */
        static void addCheck(std::function<bool()> check);

        /**
         * Updates the watchdog.
         */
        static void update();

        /**
         * Resets the watchdog.
         */
        static void reset();

        /**
         * Returns true if the robot should abort autonomous.
         */
        static bool shouldAbort();

    private:
        static bool isAborted;
        static std::vector<std::function<bool()>> checks;
    };
}