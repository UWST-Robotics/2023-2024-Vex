#pragma once
#include <functional>
#include "logger.hpp"

namespace devils
{
    class Watchdog
    {
    public:
        /**
         * Adds a check to the watchdog.
         * @param check A function that returns true if the robot is unsafe.
         */
        static void addCheck(std::function<bool()> check)
        {
            checks.push_back(check);
        }

        /**
         * Updates the watchdog.
         */
        static void update()
        {
            for (auto check : checks)
            {
                if (check())
                {
                    Logger::error("Watchdog: Robot is unsafe, sending abort signal");
                    isAborted = true;
                    return;
                }
            }
        }

        /**
         * Resets the watchdog.
         */
        static void reset()
        {
            isAborted = false;
        }

        /**
         * Returns true if the robot should abort autonomous.
         */
        static bool shouldAbort()
        {
            return isAborted;
        }

    private:
        static bool isAborted;
        static std::vector<std::function<bool()>> checks;
    };
}