#pragma once
#include "devils/utils/logger.hpp"
#include "devils/hardware/smartMotor.hpp"
#include "devils/hardware/opticalSensor.hpp"
#include "devils/hardware/scuffPneumatic.hpp"

namespace devils
{
    /**
     * Controls the intake system to intake triballs.
     */
    class IntakeSystem
    {
    public:
        /**
         * Controls the intake system to intake triballs.
         * @param intakePort The port of the intake motor.
         * @param conveyorPort The port of the conveyor motor.
         */
        IntakeSystem(std::initializer_list<int8_t> intakePorts)
            : intakeMotors("Intake Motor", intakePorts)
        {
        }

        /**
         * Runs the intake wheels.
         * @param value The speed of the intake wheels (-1 to 1).
         */
        void intake(double value = WHEEL_SPEED)
        {
            intakeMotors.moveVoltage(value);
        }

        /**
         * Runs the intake in reverse.
         */
        void outtake()
        {
            intakeMotors.moveVoltage(-WHEEL_SPEED);
        }

        /**
         * Retracts (if AUTO_RETRACT) and stops the intake.
         */
        void stop()
        {
            intakeMotors.stop();
        }

        /**
         * Enables the Optical Sensor for the intake.
         * Toggles the intake if the Optical Sensor detects a triball.
         * @param sensor The Optical Sensor to use.
         */
        void useSensor(OpticalSensor *sensor)
        {
            this->sensor = sensor;
        }

    private:
        static constexpr double WHEEL_SPEED = 1.0;
        static constexpr double SENSOR_THRESHOLD = 0.9;

        SmartMotorGroup intakeMotors;
        OpticalSensor *sensor = nullptr;
    };
}