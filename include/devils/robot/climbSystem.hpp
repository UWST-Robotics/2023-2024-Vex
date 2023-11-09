#pragma once
#include "pros/motors.hpp"

namespace devils
{
    class ClimbSystem
    {
    public:
        /**
         * Controls the climbing system to pull the robot up.
         * @param motorPort The port of the climbing motor
         */
        ClimbSystem(int8_t motorPort)
            : climbMotor(motorPort)
        {
            if (errno != 0)
                Logger::error("ClimbSystem: motor port is invalid");
        }

        /**
         * Runs the catapult motor
         */
        void climb()
        {
            climbMotor.move(MOTOR_SPEED);
            isClimbing = true;
            isDropping = false;
        }

        /**
         * Stops the catapult motor
         */
        void stop()
        {
            climbMotor.move(0);
            isClimbing = false;
            isDropping = false;
        }

        /**
         * Drops the robot
         */
        void drop()
        {
            climbMotor.move(-MOTOR_SPEED);
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
        const int8_t MOTOR_SPEED = 127;

        bool isClimbing = false;
        bool isDropping = false;
        pros::Motor climbMotor;
    };
}