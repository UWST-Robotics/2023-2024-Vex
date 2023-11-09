#pragma once
#include "pros/motors.hpp"
#include <errno.h>
#include "devils/utils/logger.hpp"

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
        IntakeSystem(int8_t wheelPort, int8_t manipPort)
            : wheelMotor(wheelPort),
              manipMotor(manipPort)
        {
            if (errno != 0)
                Logger::error("IntakeSystem: motor port is invalid");
        }

        /**
         * Pops out the intake
         */
        void extend()
        {
            manipMotor.move(MANIP_SPEED);
            isExtended = true;
        }

        /**
         * Retracts the intake
         */
        void retract()
        {
            manipMotor.move(-MANIP_SPEED);
            isExtended = false;
        }

        /**
         * Runs the intake
         */
        void intake()
        {
            wheelMotor.move(WHEEL_SPEED);
            isIntaking = true;
            isOuttaking = false;
        }

        /**
         * Runs the intake in reverse
         */
        void outtake()
        {
            wheelMotor.move(-WHEEL_SPEED);
            isIntaking = false;
            isOuttaking = true;
        }

        /**
         * Stops the intake
         */
        void stop()
        {
            wheelMotor.move(0);
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
        const int8_t WHEEL_SPEED = 127;
        const int8_t MANIP_SPEED = 127;

        bool isExtended = false;
        bool isIntaking = false;
        bool isOuttaking = false;
        pros::Motor wheelMotor;
        pros::Motor manipMotor;
    };
}