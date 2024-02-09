#pragma once

#include "../devils.h"

namespace devils
{
    /**
     * Represents an autonomous test robot
     */
    struct AutoTest : public Robot
    {
    public:
        /**
         * Creates a new instance of Blaze.
         */
        AutoTest()
            : chassis(L_MOTOR_PORTS, R_MOTOR_PORTS),
              odometry(WHEEL_RADIUS, WHEEL_BASE, TICKS_PER_REVOLUTION)
        {
            // Generate Profile
            SplineGenerator splineGenerator = SplineGenerator();
            splineGenerator.generate(&motionProfile);
        }

        void autonomous()
        {
        }

        void opcontrol()
        {
            // Teleop Controller
            pros::Controller master(pros::E_CONTROLLER_MASTER);

            // Generate Profile
            SplineGenerator splineGenerator = SplineGenerator();
            splineGenerator.generate(&motionProfile);

            // Display
            OdomRenderer odomRenderer(&odometry);
            MotionRenderer motionRenderer(&motionProfile);
            FieldRenderer fieldRenderer;

            Display teleopDisplay = Display({&odomRenderer, &motionRenderer, &fieldRenderer});

            // PID
            // PID pidController = PID(0.06, 0, 0); // <-- Translation
            // PID pidController = PID(0.6, 0, 0); // <-- Rotation
            // PID pidController = PID(0.015, 0.0002, -0.05); // <-- Test motor

            // Loop
            while (true)
            {
                // Exit if the A button is pressed

                double leftX = master.get_analog(ANALOG_LEFT_X) / 127.0;
                double leftY = master.get_analog(ANALOG_LEFT_Y) / 127.0;
                bool escape = master.get_digital(DIGITAL_A);
                if (escape)
                    break;

                // Drive
                chassis.move(leftY, leftX);

                // Odometry
                odometry.update(&chassis);

                // Display
                teleopDisplay.update();

                // Delay
                pros::delay(20);
            }

            // Stop the robot
            chassis.stop();
        }

        // Subsystems
        TankChassis chassis;
        TankWheelOdometry odometry;
        MotionProfile motionProfile;

    private:
        // V5 Ports
        static constexpr std::initializer_list<int8_t> L_MOTOR_PORTS = {1};
        static constexpr std::initializer_list<int8_t> R_MOTOR_PORTS = {2};

        // Odometry
        static constexpr double WHEEL_RADIUS = 1.625;                         // in
        static constexpr double WHEEL_BASE = 12.0;                            // in
        static constexpr double TICKS_PER_REVOLUTION = 300.0 * (60.0 / 36.0); // ticks
    };
}