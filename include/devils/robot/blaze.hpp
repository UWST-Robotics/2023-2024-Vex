#pragma once
#include "../utils/logger.hpp"
#include "../chassis/tankChassis.hpp"
#include "../odom/tankWheelOdometry.hpp"
#include "../path/motionProfile.hpp"
#include "catapultSystem.hpp"
#include "intakeSystem.hpp"
#include "wingSystem.hpp"
#include "ledSystem.hpp"

namespace devils
{
    struct Blaze
    {
    public:
        /**
         * Represents Blaze the robot and all of its subsystems.
         */
        Blaze()
            : chassis(L_MOTOR_PORTS, R_MOTOR_PORTS),
              odometry(WHEEL_RADIUS, WHEEL_BASE, TICKS_PER_REVOLUTION),
              catapult(CATAPULT_MOTOR_PORT),
              intake(INTAKE_MOTOR_PORT, MANIP_MOTOR_PORT),
              wings(LEFT_WING_PORT, RIGHT_WING_PORT),
              motionProfile(MAX_VELOCITY, MAX_ACCELERATION, MAX_JERK, WHEEL_BASE),
              leds()
        {
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
        CatapultSystem catapult;
        IntakeSystem intake;
        WingSystem wings;
        MotionProfile motionProfile;
        LEDSystem leds;

    private:
        static constexpr std::initializer_list<std::int8_t> L_MOTOR_PORTS = {1};
        static constexpr std::initializer_list<std::int8_t> R_MOTOR_PORTS = {2};

        static constexpr uint8_t LEFT_WING_PORT = 1;  // ADI
        static constexpr uint8_t RIGHT_WING_PORT = 2; // ADI
        static constexpr int8_t CATAPULT_MOTOR_PORT = 5;
        static constexpr int8_t INTAKE_MOTOR_PORT = 6;
        static constexpr int8_t MANIP_MOTOR_PORT = 7;

        static constexpr double WHEEL_RADIUS = 3.25;          // in
        static constexpr double WHEEL_BASE = 12.0;            // in
        static constexpr double TICKS_PER_REVOLUTION = 540.0; // ticks

        static constexpr double MAX_VELOCITY = 24.0;     // in per second
        static constexpr double MAX_ACCELERATION = 48.0; // in per second squared
        static constexpr double MAX_JERK = 96.0;         // in per second cubed
    };
}