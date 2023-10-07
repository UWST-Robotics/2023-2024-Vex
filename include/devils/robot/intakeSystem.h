#pragma once
#include "pros/motors.hpp"

namespace devils
{
    class IntakeSystem
    {
    public:
        /**
         * Controls the pneumatic wings that pop out to the sides of the robot.
         * @param wheelPort The port of the flywheel motor.
         * @param manipPort The port of the articulation motor.
         */
        IntakeSystem(uint8_t wheelPort, uint8_t manipPort);

        /**
         * Pops out the intake
         */
        void extend();

        /**
         * Retracts the intake
         */
        void retract();

        /**
         * Runs the intake
         */
        void intake();

        /**
         * Runs the intake in reverse
         */
        void outtake();

        /**
         * Stops the intake
         */
        void stop();

        /**
         * Returns true if the intake is extended.
         */
        const bool getExtended();

        /**
         * Returns true if the intake is intaking.
         */
        const bool getIntaking();

        /**
         * Returns true if the intake is outtaking.
         */
        const bool getOuttaking();

    private:
        const int8_t WHEEL_SPEED = 127;
        const int8_t MANIP_SPEED = 127;

        bool isExtended = false;
        bool isIntaking = false;
        bool isOuttaking = false;
        pros::Motor wheelMotor;
        pros::Motor manipMotor;
    };
}