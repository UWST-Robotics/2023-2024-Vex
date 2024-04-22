#pragma once
#include "../../hardware/smartMotor.hpp"
#include "../../hardware/scuffPneumatic.hpp"
#include "../../hardware/led.hpp"
#include "../../utils/pid.hpp"

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
         * @param name The name of the launcher system (for logging purposes)
         * @param leftMotorPort The port of the left launcher motor
         * @param rightMotorPort The port of the right launcher motor
         */
        LauncherSystem(
            std::string name,
            const int8_t leftMotorPort,
            const int8_t rightMotorPort)
            : leftMotor(name + ".LeftFlywheel", leftMotorPort),
              rightMotor(name + ".RightFlywheel", rightMotorPort)
        {
            leftMotor.setBrakeMode(false);
            rightMotor.setBrakeMode(false);
        }

        /**
         * Runs the launcher flywheels using a closed-loop PID controller.
         */
        void firePID()
        {
            // Current Velocity
            double leftMotorVelocity = leftMotor.getVelocity();
            double rightMotorVelocity = rightMotor.getVelocity();

            // Target Velocity
            double leftMotorSetpoint = flywheelSetpoint + deltaVelocity;
            double rightMotorSetpoint = -flywheelSetpoint - deltaVelocity;

            // Calculate PID
            double leftMotorVoltage = flywheelLeftPID.update(leftMotorVelocity, leftMotorSetpoint);
            double rightMotorVoltage = flywheelRightPID.update(rightMotorVelocity, rightMotorSetpoint);

            // Debug
            if (isAtSpeed())
                debugLED.enable();
            else
                debugLED.disable();

            // Run Motors
            fireVoltage(leftMotorVoltage, rightMotorVoltage);
        }

        /**
         * Runs the launcher flywheels using an open-loop voltage control.
         * @param leftVoltage The voltage to run the left motor at (from -1 to 1)
         * @param rightVoltage The voltage to run the right motor at (from -1 to 1)
         */
        void fireVoltage(double leftVoltage = 1.0, double rightVoltage = -1.0)
        {
            leftMotor.moveVoltage(leftVoltage);
            rightMotor.moveVoltage(rightVoltage);
        }

        /**
         * Stops the launcher motors
         */
        void stop()
        {
            leftMotor.stop();
            rightMotor.stop();
            debugLED.disable();

            // Reset PID
            flywheelLeftPID.reset();
            flywheelRightPID.reset();

            isFiring = false;
        }

        /**
         * Returns true if the launcher is firing.
         */
        const bool getFiring()
        {
            return isFiring;
        }

        /**
         * Sets the flywheel setpoint.
         * @param setpoint The flywheel setpoint, in RPM
         */
        void setSetpoint(double setpoint)
        {
            flywheelSetpoint = setpoint;
        }

        /**
         * Sets the delta speed.
         * @param delta The delta speed, in delta RPM
         */
        void setDelta(double delta)
        {
            deltaVelocity = delta;
        }

        /**
         * Returns the delta speed.
         * @return The delta speed, in delta RPM
         */
        double getDelta()
        {
            return deltaVelocity;
        }

        /**
         * Returns the flywheel setpoint.
         * @return The flywheel setpoint, in RPM
         */
        double getSetpoint()
        {
            return flywheelSetpoint;
        }

        /**
         * Gets the average velocity of the flywheel motors.
         */
        double getCurrentVelocity()
        {
            double leftMotorVelocity = std::abs(leftMotor.getVelocity());
            double rightMotorVelocity = std::abs(rightMotor.getVelocity());
            return (leftMotorVelocity + rightMotorVelocity) / 2;
        }

        /**
         * Returns true if the launcher is at speed.
         * @return True if the launcher is at speed, false otherwise
         */
        bool isAtSpeed()
        {
            // Get Motor Velocities
            double leftMotorDelta = std::abs(leftMotor.getVelocity()) - flywheelSetpoint;
            double rightMotorDelta = std::abs(rightMotor.getVelocity()) - flywheelSetpoint;

            // Check if the motors are at speed
            bool leftAtSpeed = std::abs(leftMotorDelta) < AT_SPEED_RANGE;
            bool rightAtSpeed = std::abs(rightMotorDelta) < AT_SPEED_RANGE;

            return leftAtSpeed && rightAtSpeed;
        }

    private:
        static constexpr double AT_SPEED_RANGE = 10;             // RPM
        static constexpr double DEFAULT_FLYWHEEL_SETPOINT = 110; // rpm
        static constexpr double DEFAULT_DELTA = 0;               // rpm

        PID flywheelLeftPID = PID(0.003, 0.0004, 0, 0.6);
        PID flywheelRightPID = PID(0.003, 0.0004, 0, 0.6);

        double flywheelSetpoint = DEFAULT_FLYWHEEL_SETPOINT;
        double deltaVelocity = DEFAULT_DELTA; // Difference between left and right motor speeds
        bool isFiring = false;
        int startTime = 0;

        SmartMotor leftMotor;
        SmartMotor rightMotor;
        LED debugLED = LED("DebugLED", 1);
    };
}