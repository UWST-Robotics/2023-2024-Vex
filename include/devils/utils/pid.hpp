#pragma once
#include "pros/rtos.hpp"

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
            goal = 0;
            lastError = 0;
            lastTime = _getCurrentTime();
        }

        /**
         * Sets the goal of the PID controller.
         * @param goal The goal of the PID controller.
         */
        void setGoal(const double goal)
        {
            this->goal = goal;
            this->lastTime = _getCurrentTime();
        }

        /**
         * Calculates the output of the PID controller.
         * @param currentValue The current value of the system.
         * @return The output of the PID controller.
         */
        const double update(const double currentValue)
        {
            double currentTime = _getCurrentTime();
            double deltaTime = currentTime - lastTime;
            lastTime = currentTime;

            double error = goal - currentValue;
            double pOut = pGain * error;
            double iOut = iGain * error * deltaTime;
            double dOut = dGain * (error - lastError) / deltaTime;
            lastError = error;

            return pOut + iOut + dOut;
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
        const double pGain;
        const double iGain;
        const double dGain;

        double lastTime = 0;
        double lastError = 0;
        double goal = 0;
    };
}