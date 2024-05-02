#pragma once
#include "devils/utils/logger.hpp"
#include "devils/hardware/smartMotorGroup.hpp"
#include "devils/hardware/opticalSensor.hpp"
#include "devils/hardware/scuffPneumaticGroup.hpp"

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
         * @param intakePorts The motor ports of the intake motors.
         * @param pneumaticPorts The ADI ports of the intake pneumatic pistons.
         */
        IntakeSystem(std::string name,
                     std::initializer_list<int8_t> intakePorts,
                     std::initializer_list<uint8_t> pneumaticPorts = {})
            : intakeMotors(name + ".Motors", intakePorts),
              intakePneumatics(name + ".Pneumatics", pneumaticPorts)
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
         * Extends the intake pneumatics.
         */
        void extend()
        {
            intakePneumatics.extend();
        }

        /**
         * Retracts the intake pneumatics.
         */
        void retract()
        {
            intakePneumatics.retract();
        }

        /**
         * Checks if the intake is extended.
         * @return True if the intake is extended, false otherwise.
         */
        bool getExtended()
        {
            return intakePneumatics.getExtended();
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
        ScuffPneumaticGroup intakePneumatics;
        OpticalSensor *sensor = nullptr;
    };
}