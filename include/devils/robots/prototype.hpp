#pragma once

#include "../devils.h"

namespace devils
{
    /**
     * Represents a prototype robot and all of its subsystems.
     */
    struct Prototype : public Robot
    {
    public:
        /**
         * Creates a new instance of Blaze.
         */
        Prototype()
            : launcher(LEFT_LAUNCHER_PORT, RIGHT_LAUNCHER_PORT, TEST_MOTOR_PORT)
        {
        }

        void autonomous()
        {
        }

        void opcontrol()
        {
            // Teleop Controller
            pros::Controller master(pros::E_CONTROLLER_MASTER);

            // PID pidController = PID(0.017, 0, 0.5);
            PID pidController = PID(0.015, 0.0002, -0.05);

            double maxValue = 0.5;
            bool wasUpPressed = false;
            bool wasDownPressed = false;

            // Loop
            while (true)
            {
                bool upPressed = master.get_digital(DIGITAL_UP);
                bool downPressed = master.get_digital(DIGITAL_DOWN);
                double inputValue = master.get_analog(ANALOG_LEFT_Y) / 127.0;
                double goalValue = inputValue * 200;

                double rpm = 0; // testMotor.getSpeed();
                double output = pidController.update(goalValue - rpm);

                Logger::info("Goal=" + std::to_string(goalValue) + ", RPM=" + std::to_string(rpm) + ", Output=" + std::to_string(output));

                if (inputValue > maxValue)
                    inputValue = maxValue;
                if (inputValue < -maxValue)
                    inputValue = -maxValue;

                if (upPressed && !wasUpPressed)
                    maxValue += 0.05;
                if (downPressed && !wasDownPressed)
                    maxValue -= 0.05;

                wasUpPressed = upPressed;
                wasDownPressed = downPressed;

                master.set_text(0, 0, std::to_string(maxValue));

                // testMotor.moveVoltage(output);
                launcher.fire(inputValue);

                pros::delay(20);
            }
        }

        // Subsystems
        LauncherSystem launcher;

    private:
        // V5 Motors
        static constexpr uint8_t LEFT_LAUNCHER_PORT = 1;
        static constexpr uint8_t RIGHT_LAUNCHER_PORT = 2;
        static constexpr uint8_t TEST_MOTOR_PORT = 3;
    };
}