#pragma once
#include "../utils/logger.hpp"
#include "../chassis/tankChassis.hpp"
#include "../odom/tankWheelOdometry.hpp"
#include "../path/motionProfile.hpp"
#include "launcherSystem.hpp"
#include "climbSystem.hpp"
#include "intakeSystem.hpp"
#include "wingSystem.hpp"
#include "blockerSystem.hpp"
#include "ledSystem.hpp"
#include "../path/squigglesGenerator.hpp"
#include "../path/splineGenerator.hpp"

namespace devils
{
    /**
     * Represents the Blaze robot and all of its subsystems.
     */
    struct Blaze
    {
    public:
        /**
         * Creates a new instance of Blaze.
         */
        Blaze()
            : chassis(L_MOTOR_PORTS, R_MOTOR_PORTS),
              imu("Blaze.IMU", IMU_PORT),
              storageSensor("Blaze.StorageSensor", STORAGE_SENSOR_PORT),
              catapult(CATAPULT_MOTOR_PORT, WINCH_MOTOR_PORT, CONVEYOR_MOTOR_PORT),
              odometry(WHEEL_RADIUS, WHEEL_BASE, TICKS_PER_REVOLUTION)
        {
            odometry.useIMU(&imu);
            catapult.useSensor(&storageSensor);

            // Motion Profile
            Logger::info("Generating Motion Profile...");
            auto generator = SplineGenerator();
            generator.generate(&motionProfile);
        }

        /**
         * Runs the robot during autonomous.
         */
        void autonomous()
        {
            Logger::info("Starting autocontrol");
            double lastTime = pros::millis();
            double timer = 1500;

            // Loop
            while (true)
            {
                double deltaTime = pros::millis() - lastTime;
                lastTime = pros::millis();
                timer -= deltaTime;

                if (timer > 0)
                    robot->catapult.extend();
                else
                    robot->catapult.stopWinch();

                robot->catapult.fire();

                // Delay to prevent the CPU from being overloaded
                pros::delay(20);
            }
        }

        /**
         * Runs the robot during operator control.
         */
        void teleoperated()
        {
            Logger::info("Starting opcontrol");

            // Teleop Controller
            pros::Controller master(pros::E_CONTROLLER_MASTER);

            // Display
            OdomRenderer odomRenderer(&robot->odometry);
            MotionRenderer motionRenderer(&robot->motionProfile);
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
                    robot->launcher.forceFire();
                else
                    robot->launcher.stopLauncher();

                if (extendCatapult)
                    robot->launcher.extend();
                else if (retractCatapult)
                    robot->launcher.retract();
                else
                    robot->launcher.stopWinch();

                // Arcade Drive
                robot->chassis.move(leftY, leftX);

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
        static constexpr uint8_t CATAPULT_MOTOR_PORT = 6;
        static constexpr uint8_t WINCH_MOTOR_PORT = 15;
        static constexpr uint8_t CONVEYOR_MOTOR_PORT = 10;

        // V5 Sensors
        static constexpr uint8_t IMU_PORT = 20;
        static constexpr uint8_t STORAGE_SENSOR_PORT = 1;

        // Odometry
        static constexpr double WHEEL_RADIUS = 1.625;                         // Radius of the wheel in inches
        static constexpr double WHEEL_BASE = 15.0;                            // Width of the robot in inches
        static constexpr double TICKS_PER_REVOLUTION = 300.0 * (60.0 / 36.0); // Number of ticks per revolution of the wheel * gear ratio
    };
}