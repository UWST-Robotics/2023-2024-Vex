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
        CatapultSystem(int8_t motorPort)
            : catapultMotor(motorPort)
        {
            if (errno != 0)
                Logger::error("IntakeSystem: motor port is invalid");
        }

        /**
         * Runs the catapult motor
         */
        void fire()
        {
            catapultMotor.move(MOTOR_SPEED);
            isFiring = true;
        }

        /**
         * Stops the catapult motor
         */
        void stop()
        {
            catapultMotor.move(0);
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
        const int8_t MOTOR_SPEED = 127;

        bool isFiring = false;
        pros::Motor catapultMotor;
    };
}