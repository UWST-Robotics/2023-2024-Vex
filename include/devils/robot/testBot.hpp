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
              motionProfile(MAX_VELOCITY, MAX_ACCELERATION, MAX_JERK, WHEEL_BASE),
              imu("TestBot.IMU", IMU_PORT),
              testPneumatic("TestBot.TestPneumatic", MANIP_PNEUMATIC_PORT)
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

        // Subsystems
        TankChassis chassis;
        TankWheelOdometry odometry;
        MotionProfile motionProfile;
        ScuffPneumatic testPneumatic;

        // Extra Sensors
        IMU imu;

    private:
        // V5 Ports
        static constexpr std::initializer_list<int8_t> L_MOTOR_PORTS = {-1, 2, -11, 12};
        static constexpr std::initializer_list<int8_t> R_MOTOR_PORTS = {-9, 10, -19, 20};
        static constexpr uint8_t INTAKE_MOTOR_PORT = 3;
        static constexpr uint8_t IMU_PORT = 18;
        // static constexpr uint8_t STORAGE_SENSOR_PORT = 18;

        // ADI Ports
        static constexpr uint8_t MANIP_PNEUMATIC_PORT = 1;

        // Drivetrain
        static constexpr double WHEEL_RADIUS = 3.25;          // in
        static constexpr double WHEEL_BASE = 12.0;            // in
        static constexpr double TICKS_PER_REVOLUTION = 400.0; // 540.0; // ticks

        // Autonomous
        static constexpr double MAX_VELOCITY = 24.0;     // in per second
        static constexpr double MAX_ACCELERATION = 48.0; // in per second squared
        static constexpr double MAX_JERK = 96.0;         // in per second cubed
    };
}