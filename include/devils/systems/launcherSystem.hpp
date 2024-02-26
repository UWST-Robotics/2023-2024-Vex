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
         * Runs the launcher motors at the default speed
         */
        void fire()
        {
            fire(flywheelSpeed);
        }

        /**
         * Runs the launcher motors at the given speed
         * @param speed The speed to run the launcher motors
         */
        void fire(double speed)
        {
            // Flywheels
            leftMotor.moveVoltage(speed + deltaSpeed);
            rightMotor.moveVoltage(-speed + deltaSpeed);
            isFiring = true;
        }

        /**
         * Runs the launcher motors at the given speed
         * and automatically extends and retracts the arm.
         */
        void autoFire()
        {
            fire();

            // Arm
            if (isArmUp)
                raiseArm();
            else
                lowerArm();

            // Handle Arm Timer
            int duration = isArmUp ? ARM_UP_TIME : ARM_DOWN_TIME;
            int deltaTime = pros::millis() - startTime;
            if (deltaTime > duration)
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
            raiseArm();
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
         * Raises the arm.
         */
        void raiseArm()
        {
            armPneumatic.extend();
            isArmUp = true;
        }

        /**
         * Lowers the arm.
         */
        void lowerArm()
        {
            armPneumatic.retract();
            isArmUp = false;
        }

        /**
         * Returns true if the arm is up.
         */
        const bool getArmUp()
        {
            return isArmUp;
        }

        /**
         * Increases the flywheel speed.
         * @return The new flywheel speed
         */
        double increaseSpeed()
        {
            flywheelSpeed += FLYWHEEL_INCREMENT;
            return flywheelSpeed;
        }

        /**
         * Decreases the flywheel speed.
         * @return The new flywheel speed
         */
        double decreaseSpeed()
        {
            flywheelSpeed -= FLYWHEEL_INCREMENT;
            return flywheelSpeed;
        }

        /**
         * Increases the delta speed.
         * @return The new delta speed
         */
        double increaseDelta()
        {
            deltaSpeed += DELTA_INCREMENT;
            return deltaSpeed;
        }

        /**
         * Decreases the delta speed.
         * @return The new delta speed
         */
        double decreaseDelta()
        {
            deltaSpeed -= DELTA_INCREMENT;
            return deltaSpeed;
        }

        /**
         * Returns the delta speed.
         * @return The delta speed
         */
        double getDelta()
        {
            return deltaSpeed;
        }

        /**
         * Returns the current flywheel speed.
         * @return The flywheel speed
         */
        double getSpeed()
        {
            return flywheelSpeed;
        }

    private:
        static constexpr double DEFAULT_FLYWHEEL_SPEED = 0.65;
        static constexpr double DEFAULT_DELTA = 0.2;
        static constexpr double FLYWHEEL_INCREMENT = 0.05;
        static constexpr double DELTA_INCREMENT = 0.05;
        static constexpr int ARM_DOWN_TIME = 500; // ms
        static constexpr int ARM_UP_TIME = 2000;  // ms

        double flywheelSpeed = DEFAULT_FLYWHEEL_SPEED;
        double deltaSpeed = DEFAULT_DELTA; // Difference between left and right motor speeds
        bool isFiring = false;
        bool isArmUp = false;
        int startTime = 0;

        SmartMotor leftMotor;
        SmartMotor rightMotor;
        ScuffPneumatic armPneumatic;
    };
}