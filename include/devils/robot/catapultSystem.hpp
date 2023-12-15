#pragma once
#include "../hardware/smartMotor.hpp"

namespace devils
{
    /**
     * Controls the catapult system to launch triballs.
     */
    class CatapultSystem
    {
    public:
        /**
         * Creates a new catapult system.
         * @param motorPort The port of the catapult motor
         * @param winchPort The port of the winch motor
         */
        CatapultSystem(const int8_t motorPort, const int8_t winchPort)
            : catapultMotor("CatapultMotor", motorPort),
              winchMotor("WinchMotor", winchPort)
        {
        }

        /**
         * Runs the catapult motor
         */
        void fire()
        {
            catapultMotor.moveVoltage(FIRE_SPEED);
            isFiring = true;
        }

        /**
         * Stops the catapult motor
         */
        void stopLauncher()
        {
            catapultMotor.moveVoltage(0);
            isFiring = false;
        }

        /**
         * Stops the winch motor
         */
        void stopWinch()
        {
            winchMotor.moveVoltage(0);
        }

        /**
         * Stops both the catapult and winch motors
         */
        void stop()
        {
            stopLauncher();
            stopWinch();
        }

        /**
         * Extends the winch
         */
        void extend()
        {
            winchMotor.moveVoltage(WINCH_SPEED);
        }

        /**
         * Retracts the winch
         */
        void retract()
        {
            winchMotor.moveVoltage(-WINCH_SPEED);
        }

        /**
         * Returns true if the catapult is firing.
         */
        const bool getFiring()
        {
            return isFiring;
        }

    private:
        static constexpr double FIRE_SPEED = 1.0;
        static constexpr double WINCH_SPEED = 0.5;

        bool isFiring = false;
        SmartMotor catapultMotor;
        SmartMotor winchMotor;
    };
}