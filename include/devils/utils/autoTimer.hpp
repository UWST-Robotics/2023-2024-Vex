#pragma once

#include "pros/rtos.hpp"

namespace devils
{
    class AutoTimer
    {
    public:
        /**
         * Starts the timer if it is not already running.
         * @param timerID The ID of the timer.
         * @param duration The duration of the timer in milliseconds.
         */
        void start(std::string timerID, double duration)
        {
            if (lastTimerID == timerID)
                return;

            lastTimerID = timerID;
            isRunning = true;
            startTime = pros::millis();
            this->duration = duration;
        }

        /**
         * Stops the timer.
         */
        void stop()
        {
            isRunning = false;
        }

        /**
         * Returns true if the timer is running.
         */
        bool getRunning()
        {
            if (isRunning && pros::millis() - startTime > duration)
                isRunning = false;
            return isRunning;
        }

    private:
        std::string lastTimerID = "";
        bool isRunning = false;
        double startTime = 0;
        double duration = 0;
    };
}