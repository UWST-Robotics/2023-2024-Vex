#pragma once
#include "../devils.h"
#include "control/pjAutoController.hpp"
#include <time.h>

namespace devils
{
    /**
     * Represents an autonomous test robot
     */
    struct BlazeTest : public Robot
    {
        /**
         * Creates a new instance of Auto Tester.
         */
        BlazeTest()
        {
            displayStack.runAsync();
        }

        void opcontrol() override
        {
            // Loop
            while (true)
            {
                double inputA = mainController.get_analog(ANALOG_LEFT_Y) / 127.0;
                double inputB = mainController.get_analog(ANALOG_RIGHT_Y) / 127.0;
                motorsA.moveVoltage(inputA);
                motorsB.moveVoltage(inputB);

                // Intake
                if (mainController.get_digital(DIGITAL_L1) || mainController.get_digital(DIGITAL_R1))
                {
                    intake1.extend();
                    intake2.extend();
                    intake3.extend();
                    intake4.extend();
                }
                else if (mainController.get_digital(DIGITAL_L2) || mainController.get_digital(DIGITAL_R2))
                {
                    intake1.retract();
                    intake2.retract();
                    intake3.retract();
                    intake4.retract();
                }

                // Delay
                pros::delay(20);
            }
        }

    private:
        // Motors
        SmartMotorGroup motorsA = SmartMotorGroup("TestMotors", {1, 2, 3, 4, 5, -6, -7, -8, -9, -10});
        SmartMotorGroup motorsB = SmartMotorGroup("TestMotors", {11, 12, 13, 14, 15, -16, -17, -18, -19, -20});

        ScuffPneumatic intake1 = ScuffPneumatic("Intake1", 1);
        ScuffPneumatic intake2 = ScuffPneumatic("Intake2", 2);
        ScuffPneumatic intake3 = ScuffPneumatic("Intake3", 3);
        ScuffPneumatic intake4 = ScuffPneumatic("Intake4", 4);

        // Display
        Display displayStack = Display({new StatsRenderer()});
        StatsRenderer *statsRenderer = displayStack.getRenderer<StatsRenderer>();
    };
}