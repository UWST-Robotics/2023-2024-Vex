#pragma once

#include "../devils.h"

namespace devils
{
    /**
     * Represents the Blaze robot and all of its subsystems.
     */
    struct Blaze : public Robot
    {
    public:
        /**
         * Creates a new instance of Blaze.
         */
        Blaze()
            : chassis(L_MOTOR_PORTS, R_MOTOR_PORTS),
              imu("Blaze.IMU", IMU_PORT),
              storageSensor("Blaze.StorageSensor", STORAGE_SENSOR_PORT),
              launcher(LEFT_LAUNCHER_PORT, RIGHT_LAUNCHER_PORT),
              odometry(WHEEL_RADIUS, WHEEL_BASE, TICKS_PER_REVOLUTION)
        {
            odometry.useIMU(&imu);
            // launcher.useSensor(&storageSensor);

            // Motion Profile
            Logger::info("Generating Motion Profile...");
            auto generator = SplineGenerator();
            generator.generate(&motionProfile);
        }

        void autonomous()
        {
            // Loop
            while (true)
            {
                // TODO: Write me!

                // Delay to prevent the CPU from being overloaded
                pros::delay(20);
            }
        }

        void opcontrol()
        {
            Logger::info("Starting opcontrol");

            // Teleop Controller
            pros::Controller master(pros::E_CONTROLLER_MASTER);

            // Display
            OdomRenderer odomRenderer(&odometry);
            MotionRenderer motionRenderer(&motionProfile);
            Display teleopDisplay = Display({&odomRenderer, &motionRenderer});

            // Loop
            while (true)
            {
                // Controller
                double leftY = master.get_analog(ANALOG_LEFT_Y) / 127.0;
                double leftX = master.get_analog(ANALOG_RIGHT_X) / 127.0;
                bool extendCatapult = master.get_digital(DIGITAL_R1);
                bool retractCatapult = master.get_digital(DIGITAL_R2);
                bool fireLauncher = master.get_digital(DIGITAL_L1);
                bool block = master.get_digital(DIGITAL_A);
                bool wings = master.get_digital(DIGITAL_B);

                // Curve Inputs
                leftY = Curve::square(Curve::dlerp(0.1, 0.3, 1.0, leftY));
                leftX = Curve::square(leftX);

                // Catapult
                if (fireLauncher)
                    launcher.fire();
                else
                    launcher.stop();

                /*
                if (extendCatapult)
                    launcher.extend();
                else if (retractCatapult)
                    launcher.retract();
                else
                    launcher.stopWinch();
                    */

                // Arcade Drive
                chassis.move(leftY, leftX);

                // Odometry
                odometry.update(&chassis);

                // Simulation
                teleopDisplay.update();

                // Delay to prevent the CPU from being overloaded
                pros::delay(20);
            }
        }

        // Subsystems
        TankChassis chassis;
        LauncherSystem launcher;

        // Autonomous
        TankWheelOdometry odometry;
        MotionProfile motionProfile;

        // Extra Sensors
        IMU imu;
        OpticalSensor storageSensor;

    private:
        // V5 Motors
        static constexpr std::initializer_list<int8_t> L_MOTOR_PORTS = {12, 4, -3, -11}; //{9, -10, 19, -20};
        static constexpr std::initializer_list<int8_t> R_MOTOR_PORTS = {-8, -16, 17, 7}; //{1, -2, 11, -12};
        static constexpr uint8_t LEFT_LAUNCHER_PORT = 5;
        static constexpr uint8_t RIGHT_LAUNCHER_PORT = 6;

        // V5 Sensors
        static constexpr uint8_t IMU_PORT = 20;
        static constexpr uint8_t STORAGE_SENSOR_PORT = 1;

        // Odometry
        static constexpr double WHEEL_RADIUS = 1.625;                         // Radius of the wheel in inches
        static constexpr double WHEEL_BASE = 15.0;                            // Width of the robot in inches
        static constexpr double TICKS_PER_REVOLUTION = 300.0 * (60.0 / 36.0); // Number of ticks per revolution of the wheel * gear ratio
    };
}