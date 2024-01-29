#pragma once
#include "../hardware/smartMotor.hpp"

namespace devils
{
    /**
     * Controls the launcher system to launch triballs.
     */
    class LauncherSystem
    {
    public:
        /**
         * Creates a new launcher system.
         * @param leftMotorPort The port of the left launcher motor
         * @param rightMotorPort The port of the right launcher motor
         */
        LauncherSystem(const int8_t leftMotorPort, const int8_t rightMotorPort)
            : leftMotor("Left Launcher Motor", leftMotorPort),
              rightMotor("Right Launcher Motor", rightMotorPort)
        {
        }

        /**
         * Runs the launcher motors
         */
        void fire(double val = MOTOR_SPEED)
        {
            leftMotor.moveVoltage(val);
            rightMotor.moveVoltage(-val);
            isFiring = true;
        }

        /**
         * Stops the launcher motors
         */
        void stop()
        {
            leftMotor.moveVoltage(0);
            rightMotor.moveVoltage(0);
            isFiring = false;
        }

        /**
         * Returns true if the launcher is firing.
         */
        const bool getFiring()
        {
            return isFiring;
        }

    private:
        static constexpr double MOTOR_SPEED = 1.0;

        bool isFiring = false;
        SmartMotor leftMotor;
        SmartMotor rightMotor;
    };
}