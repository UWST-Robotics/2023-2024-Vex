#pragma once
#include "../utils/logger.hpp"
#include "../chassis/tankChassis.hpp"
#include "../odom/tankWheelOdometry.hpp"
#include "../path/motionProfile.hpp"
#include "catapultSystem.hpp"
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
              catapult(CATAPULT_MOTOR_PORT, WINCH_MOTOR_PORT),
              odometry(WHEEL_RADIUS, WHEEL_BASE, TICKS_PER_REVOLUTION)
        {
            odometry.useIMU(&imu);
            catapult.useSensor(&storageSensor);
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
            Logger::info("Generating Motion Profile...");
            auto generator = SplineGenerator();
            generator.generate(&motionProfile);
        }

        // Subsystems
        TankChassis chassis;
        CatapultSystem catapult;

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

        // V5 Sensors
        static constexpr uint8_t IMU_PORT = 20;
        static constexpr uint8_t STORAGE_SENSOR_PORT = 1;

        // Odometry
        static constexpr double WHEEL_RADIUS = 1.625;                         // Radius of the wheel in inches
        static constexpr double WHEEL_BASE = 15.0;                            // Width of the robot in inches
        static constexpr double TICKS_PER_REVOLUTION = 300.0 * (60.0 / 36.0); // Number of ticks per revolution of the wheel * gear ratio
    };
}