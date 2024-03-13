#pragma once
#include "../hardware/smartMotor.hpp"
#include "../utils/logger.hpp"

namespace devils
{
    /**
     * Controls the climbing system to pull the robot up.
     */
    class ClimbSystem
    {
    public:
        /**
         * Creates a new climbing system.
         * @param motorPort The port of the climbing motor
         */
        ClimbSystem(const int8_t motorPort)
            : climbMotor("ClimbMotor", motorPort)
        {
        }

        /**
         * Restarts the climb timer
         */
        void restart()
        {
            startTime = pros::millis();
        }

        /**
         * Runs the catapult motor
         */
        void climb()
        {
            climbMotor.moveVoltage(MOTOR_SPEED);
            isClimbing = true;
            isDropping = false;
        }

        /**
         * Stops the catapult motor
         */
        void stop()
        {
            climbMotor.moveVoltage(0);
            isClimbing = false;
            isDropping = false;
        }

        /**
         * Drops the robot
         */
        void drop()
        {
            climbMotor.moveVoltage(-MOTOR_SPEED);
            isClimbing = false;
            isDropping = true;
        }

        /**
         * Returns true if the climber is ascending.
         */
        const bool getClimbing()
        {
            return isClimbing;
        }

        /**
         * Returns true if the climber is descending.
         */
        const bool getDropping()
        {
            return isDropping;
        }

    private:
        static constexpr double MOTOR_SPEED = 1.0;
        static constexpr int AUTO_CLIMB_TIME = 1000 * 119; // 1:59

        bool isClimbing = false;
        bool isDropping = false;
        SmartMotor climbMotor;
        int startTime = 0;
    };
}