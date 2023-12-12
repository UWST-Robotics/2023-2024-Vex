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
    /**
     * Represents Blaze the robot and all of its subsystems.
     */
    struct Blaze
    {
    public:
        /**
         * Creates a new instance of Blaze.
         */
        Blaze()
            : chassis(L_MOTOR_PORTS, R_MOTOR_PORTS),
              odometry(WHEEL_RADIUS, WHEEL_BASE, TICKS_PER_REVOLUTION),
              catapult(CATAPULT_MOTOR_PORT),
              intake(INTAKE_MOTOR_PORT, MANIP_MOTOR_PORT),
              wings(WINGS_PORT)
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

    private:
        static constexpr std::initializer_list<std::int8_t> L_MOTOR_PORTS = {1};
        static constexpr std::initializer_list<std::int8_t> R_MOTOR_PORTS = {2};

        static constexpr uint8_t WINGS_PORT = 1; // ADI
        static constexpr int8_t CATAPULT_MOTOR_PORT = 5;
        static constexpr int8_t INTAKE_MOTOR_PORT = 6;
        static constexpr int8_t MANIP_MOTOR_PORT = 7;

        static constexpr double WHEEL_RADIUS = 3.25;          // in
        static constexpr double WHEEL_BASE = 12.0;            // in
        static constexpr double TICKS_PER_REVOLUTION = 540.0; // ticks
    };
}