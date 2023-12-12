#pragma once
#include "../utils/logger.hpp"
#include "../chassis/tankChassis.hpp"
#include "../odom/tankWheelOdometry.hpp"
#include "../path/motionProfile.hpp"
#include "catapultSystem.hpp"
#include "climbSystem.hpp"
#include "intakeSystem.hpp"
#include "wingSystem.hpp"
#include "ledSystem.hpp"
#include "../path/squigglesGenerator.hpp"
#include "../path/splineGenerator.hpp"

namespace devils
{
    /**
     * Represents the testbed robot and all of its subsystems.
     */
    struct TestBot
    {
    public:
        /**
         * Creates a new instance of TestBot.
         */
        TestBot()
            : chassis(L_MOTOR_PORTS, R_MOTOR_PORTS),
              odometry(WHEEL_RADIUS, WHEEL_BASE, TICKS_PER_REVOLUTION),
              imu("TestBot.IMU", IMU_PORT),
              intake(INTAKE_MOTOR_PORT, MANIP_PNEUMATIC_PORT),
              storageSensor("TestBot.StorageSensor", STORAGE_SENSOR_PORT),
              wings(WINGS_PNEUMATIC_PORT)
        {
            odometry.useIMU(&imu);
        }

        /**
         * Updates the odometry of the robot.
         */
        void updateOdometry()
        {
            odometry.update(&chassis);
        }

        /**
         * Generates the motion profile for the robot.
         */
        void generateMotionProfile()
        {
            // auto generator = SquigglesGenerator(MAX_VELOCITY, MAX_ACCELERATION, MAX_JERK, WHEEL_BASE);
            auto generator = SplineGenerator();
            generator.generate(&motionProfile);
        }

        // Subsystems
        TankChassis chassis;
        TankWheelOdometry odometry;
        MotionProfile motionProfile;
        IntakeSystem intake;
        WingSystem wings;

        // Extra Sensors
        IMU imu;
        OpticalSensor storageSensor;

    private:
        // V5 Ports
        static constexpr std::initializer_list<int8_t> L_MOTOR_PORTS = {1};  // {-1, 2, -11, 12};
        static constexpr std::initializer_list<int8_t> R_MOTOR_PORTS = {-2}; //{-9, 10, -19, 20};
        static constexpr uint8_t INTAKE_MOTOR_PORT = 3;
        static constexpr uint8_t IMU_PORT = 17;
        static constexpr uint8_t STORAGE_SENSOR_PORT = 18;

        // ADI Ports
        static constexpr uint8_t MANIP_PNEUMATIC_PORT = 1;
        static constexpr uint8_t WINGS_PNEUMATIC_PORT = 2;

        // Drivetrain
        static constexpr double WHEEL_RADIUS = 3.25;          // in
        static constexpr double WHEEL_BASE = 12.0;            // in
        static constexpr double TICKS_PER_REVOLUTION = 400.0; // 540.0; // ticks

        // Autonomous
        static constexpr double MAX_VELOCITY = 36.0;     // in per second
        static constexpr double MAX_ACCELERATION = 48.0; // in per second squared
        static constexpr double MAX_JERK = 96.0;         // in per second cubed
    };
}