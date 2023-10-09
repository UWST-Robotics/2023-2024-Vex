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
        ClimbSystem(int8_t motorPort);

        /**
         * Runs the catapult motor
         */
        void climb();

        /**
         * Stops the catapult motor
         */
        void stop();

        /**
         * Drops the robot
         */
        void drop();

        /**
         * Returns true if the climber is ascending.
         */
        const bool getClimbing();

        /**
         * Returns true if the climber is descending.
         */
        const bool getDropping();

    private:
        const int8_t MOTOR_SPEED = 127;

        bool isClimbing = false;
        bool isDropping = false;
        pros::Motor climbMotor;
    };
}