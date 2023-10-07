#pragma once
#include "pros/motors.hpp"

namespace devils
{
    class CatapultSystem
    {
    public:
        /**
         * Controls the catapult system to launch balls.
         * @param motorPort The port of the catapult motor
         */
        CatapultSystem(uint8_t motorPort);

        /**
         * Runs the catapult motor
         */
        void fire();

        /**
         * Stops the catapult motor
         */
        void stop();

        /**
         * Returns true if the catapult is firing.
         */
        const bool getFiring();

    private:
        const int8_t MOTOR_SPEED = 127;

        bool isFiring = false;
        pros::Motor catapultMotor;
    };
}