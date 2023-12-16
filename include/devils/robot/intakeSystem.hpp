#pragma once
#include "devils/utils/logger.hpp"
#include "../hardware/smartMotor.hpp"
#include "../hardware/opticalSensor.hpp"
#include "../hardware/scuffPneumatic.hpp"

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
         * Extends (if AUTO_EXTEND) and runs the intake wheels.
         * Stops if the Optical Sensor detects a triball.
         */
        void intake()
        {
            if (enableSensor && sensor->getProximity() > SENSOR_THRESHOLD)
                stop();
            else
                forceIntake();
        }

        /**
         * Extends (if AUTO_EXTEND) and runs the intake, regardless of the Optical Sensor.
         */
        void forceIntake()
        {
            intakeMotor.moveVoltage(WHEEL_SPEED);
            isIntaking = true;
            isOuttaking = false;
        }

        /**
         * Extends (if AUTO_EXTEND) and runs the intake in reverse.
         */
        void outtake()
        {
            intakeMotor.moveVoltage(-WHEEL_SPEED);
            isIntaking = false;
            isOuttaking = true;
        }

        /**
         * Retracts (if AUTO_RETRACT) and stops the intake.
         */
        void stop()
        {
            intakeMotor.stop();
            isIntaking = false;
            isOuttaking = false;
        }

        /**
         * Enables the Optical Sensor for the intake.
         * Toggles the intake if the Optical Sensor detects a triball.
         * @param sensor The Optical Sensor to use.
         */
        void useSensor(OpticalSensor *sensor)
        {
            enableSensor = true;
            this->sensor = sensor;
        }

        /**
         * Returns true if the intake is extended.
         */
        const bool getExtended()
        {
            return isExtended;
        }

        /**
         * Returns true if the intake is intaking.
         */
        const bool getIntaking()
        {
            return isIntaking;
        }

        /**
         * Returns true if the intake is outtaking.
         */
        const bool getOuttaking()
        {
            return isOuttaking;
        }

    private:
        static constexpr double WHEEL_SPEED = 1.0;
        static constexpr double SENSOR_THRESHOLD = 0.5;

        bool isExtended = false;
        bool isIntaking = false;
        bool isOuttaking = false;
        bool enableSensor = false;
        SmartMotor intakeMotor;
        OpticalSensor *sensor;
    };
}