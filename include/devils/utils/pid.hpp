#pragma once
#include "pros/rtos.hpp"
#include "../utils/logger.hpp"
#include <ctime>

namespace devils
{
    /**
     * Represents a simple PID controller.
     */
    class PID
    {
    public:
        /**
         * Creates a new PID controller.
         * @param pGain The proportional constant.
         * @param iGain The integral constant.
         * @param dGain The derivative constant.
         */
        PID(const double pGain,
            const double iGain,
            const double dGain)
            : pGain(pGain),
              iGain(iGain),
              dGain(dGain)
        {
            lastError = 0;
            lastTime = _getCurrentTime();
            lastOutput = 0;
        }

        /**
         * Calculates the output of the PID controller.
         * @param currentError The current distance from the goal.
         * @return The output of the PID controller.
         */
        const double update(const double currentError)
        {
            // Get Delta Time
            double currentTime = _getCurrentTime();
            double deltaTime = currentTime - lastTime;

            // Abort if the delta time is too small
            if (deltaTime < MIN_DELTA_TIME)
                return lastOutput;
            lastTime = currentTime;

            double pOut = pGain * currentError;
            double iOut = iGain * currentError * deltaTime;
            double dOut = dGain * (currentError - lastError) / deltaTime;
            lastError = currentError;

            lastOutput = pOut + iOut + dOut;
            return lastOutput;
        }

        /**
         * Gets the current time in milliseconds.
         * @return The current time in milliseconds.
         */
        double _getCurrentTime()
        {
            return pros::millis();
        }

    private:
        // std::Timer timer;

        constexpr static double MIN_DELTA_TIME = 20;

        const double pGain;
        const double iGain;
        const double dGain;

        double lastTime = 0;
        double lastError = 0;
        double lastOutput = 0;
    };
}