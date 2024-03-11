#pragma once
#include "pros/motors.hpp"
#include "../utils/logger.hpp"
#include <string>

namespace devils
{
    /**
     * Represents some kind of motor or motor group.
     */
    struct IMotor
    {
        /**
         * Runs the motor in voltage mode.
         * @param voltage The voltage to run the motor at, from -1 to 1.
         */
        virtual void moveVoltage(const double voltage) = 0;

        /**
         * Stops the motor.
         */
        virtual void stop() = 0;

        /**
         * Gets the current position of the motor in encoder ticks.
         * @return The current position of the motor in encoder ticks.
         */
        virtual const double getPosition() = 0;

        /**
         * Returns the current speed of the motor in RPM.
         */
        virtual const double getSpeed() = 0;
    };
}