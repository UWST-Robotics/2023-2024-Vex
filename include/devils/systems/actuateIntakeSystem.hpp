#pragma once
#include "../utils/logger.hpp"
#include "../hardware/smartMotor.hpp"
#include "../hardware/opticalSensor.hpp"
#include "../hardware/scuffPneumatic.hpp"

namespace devils
{
    /**
     * Controls the intake system w/ a pneumatic actuator to intake triballs.
     */
    class ActuateIntakeSystem
    {
    public:
        /**
         * Controls the intake system to intake triballs.
         * @param wheelPort The port of the flywheel motor.
         * @param pneumaticPort The port of the actuator pneumatic.
         */
        ActuateIntakeSystem(const int8_t wheelPort, const uint8_t pneumaticPort)
            : intakeMotor("IntakeMotor", wheelPort),
              intakePneumatic("IntakePneumatic", pneumaticPort)
        {
        }

        /**
         * Extends (if AUTO_EXTEND) and runs the intake, regardless of the Optical Sensor.
         */
        void intake(double value = WHEEL_SPEED)
        {
            intakeMotor.moveVoltage(value);
        }

        /**
         * Extends (if AUTO_EXTEND) and runs the intake in reverse.
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
         * Extends the intake pneumatic.
         */
        void raise()
        {
            intakePneumatic.extend();
        }

        /**
         * Retracts the intake pneumatic.
         */
        void lower()
        {
            intakePneumatic.retract();
        }

    private:
        static constexpr double WHEEL_SPEED = 0.7;

        SmartMotor intakeMotor;
        ScuffPneumatic intakePneumatic;
    };
}