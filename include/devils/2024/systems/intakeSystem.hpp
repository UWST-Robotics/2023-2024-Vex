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
         * @param wheelPort The port of the flywheel motor.
         */
        IntakeSystem(const int8_t wheelPort)
            : intakeMotor("IntakeMotor", wheelPort)
        {
        }

        /**
         * Runs the intake wheels automatically using the Optical Sensor.
         */
        void autoIntake()
        {
            if (sensor != nullptr && sensor->getProximity() > SENSOR_THRESHOLD)
                intake(SENSOR_SPEED);
            else
                intake();
        }

        /**
         * Runs the intake wheels.
         * @param value The speed of the intake wheels (-1 to 1).
         */
        void intake(double value = WHEEL_SPEED)
        {
            intakeMotor.moveVoltage(value);
        }

        /**
         * Runs the intake in reverse.
         */
        void outtake()
        {
            intakeMotor.moveVoltage(-WHEEL_SPEED);
        }

        /**
         * Retracts (if AUTO_RETRACT) and stops the intake.
         */
        void stop()
        {
            intakeMotor.stop();
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
        static constexpr double WHEEL_SPEED = 0.5;
        static constexpr double SENSOR_SPEED = 0.5;
        static constexpr double SENSOR_THRESHOLD = 0.5;

        SmartMotor intakeMotor;
        OpticalSensor *sensor = nullptr;
    };
}