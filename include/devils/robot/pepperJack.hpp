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
     * Represents the Pepper Jack robot and all of its subsystems.
     */
    struct PepperJack
    {
    public:
        /**
         * Creates a new instance of PepperJack.
         */
        PepperJack()
            : chassis(L_MOTOR_PORTS, R_MOTOR_PORTS),
              imu("PepperJack.IMU", IMU_PORT),
              intake(INTAKE_MOTOR_PORT),
              storageSensor("PepperJack.StorageSensor", STORAGE_SENSOR_PORT),
              wings(LEFT_WINGS_PNEUMATIC_PORT, RIGHT_WINGS_PNEUMATIC_PORT),
              blocker(BLOCKER_PNEUMATIC_DOWN_PORT, BLOCKER_PNEUMATIC_UP_PORT),
              odometry(WHEEL_RADIUS, WHEEL_BASE, TICKS_PER_REVOLUTION)
        {
            odometry.useIMU(&imu);
            // intake.useSensor(&storageSensor);
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
        IntakeSystem intake;
        WingSystem wings;
        BlockerSystem blocker;

        // Autonomous
        TankWheelOdometry odometry;
        MotionProfile motionProfile;

        // Extra Sensors
        IMU imu;
        OpticalSensor storageSensor;

    private:
        // V5 Motors
        static constexpr std::initializer_list<int8_t> L_MOTOR_PORTS = {1, 11, -2, -12};
        static constexpr std::initializer_list<int8_t> R_MOTOR_PORTS = {-10, -20, 9, 19};
        static constexpr uint8_t INTAKE_MOTOR_PORT = -21;

        // V5 Sensors
        static constexpr uint8_t IMU_PORT = 6;
        static constexpr uint8_t STORAGE_SENSOR_PORT = 7;

        // ADI Ports
        static constexpr uint8_t RIGHT_WINGS_PNEUMATIC_PORT = 1;
        static constexpr uint8_t LEFT_WINGS_PNEUMATIC_PORT = 2;
        static constexpr uint8_t BLOCKER_PNEUMATIC_DOWN_PORT = 3;
        static constexpr uint8_t BLOCKER_PNEUMATIC_UP_PORT = 4;

        // Odometry
        static constexpr double WHEEL_RADIUS = 3.25;                          // in
        static constexpr double WHEEL_BASE = 12.0;                            // in
        static constexpr double TICKS_PER_REVOLUTION = 300.0 * (60.0 / 36.0); // ticks
    };
}