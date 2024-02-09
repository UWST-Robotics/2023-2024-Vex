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
         * @param armPneumaticPort The ADI port of the arm pneumatic
         */
        LauncherSystem(const int8_t leftMotorPort, const int8_t rightMotorPort, const int8_t armPneumaticPort)
            : leftMotor("Left Launcher Motor", leftMotorPort),
              rightMotor("Right Launcher Motor", rightMotorPort),
              armPneumatic("Arm Pneumatic", armPneumaticPort)
        {
        }

        /**
         * Runs the launcher motors
         */
        void fire(double val = FLYWHEEL_SPEED)
        {
            // Flywheels
            leftMotor.moveVoltage(val);
            rightMotor.moveVoltage(-val);
            isFiring = true;

            // Arm
            if (isArmUp)
                armPneumatic.extend();
            else
                armPneumatic.retract();

            // Handle Arm Timer
            int deltaTime = pros::millis() - startTime;
            if (deltaTime > ARM_TIME)
            {
                isArmUp = !isArmUp;
                startTime = pros::millis();
            }
        }

        /**
         * Stops the launcher motors
         */
        void stop()
        {
            leftMotor.moveVoltage(0);
            rightMotor.moveVoltage(0);
            armPneumatic.retract();
            isFiring = false;
            isArmUp = false;
        }

        /**
         * Returns true if the launcher is firing.
         */
        const bool getFiring()
        {
            return isFiring;
        }

        /**
         * Returns true if the arm is up.
         */
        const bool getArmUp()
        {
            return isArmUp;
        }

    private:
        static constexpr double FLYWHEEL_SPEED = 1.0;
        static constexpr int ARM_TIME = 500; // ms

        bool isFiring = false;
        bool isArmUp = false;

        int startTime = 0;

        SmartMotor leftMotor;
        SmartMotor rightMotor;
        ScuffPneumatic armPneumatic;
    };
}