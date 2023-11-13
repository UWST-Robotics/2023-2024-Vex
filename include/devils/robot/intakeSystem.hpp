#pragma once
#include "../hardware/smartMotor.hpp"
#include "devils/utils/logger.hpp"

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
         * @param manipPort The port of the articulation motor.
         */
        IntakeSystem(const int8_t wheelPort, const int8_t manipPort)
            : intakeMotor("IntakeMotor", wheelPort),
              manipMotor("ManipMotor", manipPort)
        {
        }

        /**
         * Pops out the intake
         */
        void extend()
        {
            manipMotor.moveVoltage(MANIP_SPEED);
            isExtended = true;
        }

        /**
         * Retracts the intake
         */
        void retract()
        {
            manipMotor.moveVoltage(-MANIP_SPEED);
            isExtended = false;
        }

        /**
         * Runs the intake
         */
        void intake()
        {
            intakeMotor.moveVoltage(WHEEL_SPEED);
            isIntaking = true;
            isOuttaking = false;
        }

        /**
         * Runs the intake in reverse
         */
        void outtake()
        {
            intakeMotor.moveVoltage(-WHEEL_SPEED);
            isIntaking = false;
            isOuttaking = true;
        }

        /**
         * Stops the intake
         */
        void stop()
        {
            intakeMotor.stop();
            isIntaking = false;
            isOuttaking = false;
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
        static constexpr double MANIP_SPEED = 1.0;

        bool isExtended = false;
        bool isIntaking = false;
        bool isOuttaking = false;
        SmartMotor intakeMotor;
        SmartMotor manipMotor;
    };
}