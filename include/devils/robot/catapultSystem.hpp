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
         */
        CatapultSystem(const int8_t motorPort)
            : catapultMotor("CatapultMotor", motorPort)
        {
        }

        /**
         * Runs the catapult motor
         */
        void fire()
        {
            catapultMotor.moveVoltage(MOTOR_SPEED);
            isFiring = true;
        }

        /**
         * Stops the catapult motor
         */
        void stop()
        {
            catapultMotor.moveVoltage(0);
            isFiring = false;
        }

        /**
         * Returns true if the catapult is firing.
         */
        const bool getFiring()
        {
            return isFiring;
        }

    private:
        static constexpr double MOTOR_SPEED = 1.0;

        bool isFiring = false;
        SmartMotor catapultMotor;
    };
}