#pragma once
#include "../../hardware/scuffPneumatic.hpp"

namespace devils
{
    /**
     * Controls the pneumatic wings that pop out to the sides of the robot.
     */
    class WingSystem
    {
    public:
        /**
         * Creates a new wing system.
         * @param name The name of the wing system (for logging purposes)
         * @param leftPort The ADI port of the left wing pneumatic.
         * @param rightPort The ADI port of the right wing pneumatic.
         */
        WingSystem(
            std::string name,
            const uint8_t leftPort,
            const uint8_t rightPort)
            : leftPneumatic(name + ".LeftWing", leftPort),
              rightPneumatic(name + ".RightWing", rightPort)
        {
        }

        /**
         * Automatically extends and retracts the left wing on a timer.
         */
        void autoExtendLeft()
        {
            // Arm
            if (isLeftExtended)
                extendLeft();
            else
                retractLeft();

            // Handle Arm Timer
            int duration = isLeftExtended ? AUTO_OPEN_TIME : AUTO_CLOSE_TIME;
            int deltaTime = pros::millis() - startTime;
            if (deltaTime > duration)
            {
                isLeftExtended = !isLeftExtended;
                startTime = pros::millis();
            }
        }

        /**
         * Automatically extends and retracts the right wing on a timer.
         */
        void autoExtendRight()
        {
            // Arm
            if (isRightExtended)
                extendRight();
            else
                retractRight();

            // Handle Arm Timer
            int duration = isRightExtended ? AUTO_OPEN_TIME : AUTO_CLOSE_TIME;
            int deltaTime = pros::millis() - startTime;
            if (deltaTime > duration)
            {
                isRightExtended = !isRightExtended;
                startTime = pros::millis();
            }
        }
        /**
         * Pops out the left wing.
         */
        void extendLeft()
        {
            leftPneumatic.extend();
            isLeftExtended = true;
        }

        /**
         * Pops out the right wing
         */
        void extendRight()
        {
            rightPneumatic.extend();
            isRightExtended = true;
        }

        /**
         * Retracts the left wing
         */
        void retractLeft()
        {
            leftPneumatic.retract();
            isLeftExtended = false;
        }

        /**
         * Retracts the right wing
         */
        void retractRight()
        {
            rightPneumatic.retract();
            isRightExtended = false;
        }

    private:
        static constexpr int AUTO_OPEN_TIME = 250;   // ms
        static constexpr int AUTO_CLOSE_TIME = 1250; // ms

        bool isLeftExtended = false;
        bool isRightExtended = false;
        int startTime = 0;
        ScuffPneumatic leftPneumatic;
        ScuffPneumatic rightPneumatic;
    };
}