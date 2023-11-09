#pragma once
#include "devils/utils/logger.hpp"
#include "devils/chassis/tankChassis.hpp"
#include "devils/odom/tankWheelOdometry.hpp"
#include "catapultSystem.hpp"
#include "intakeSystem.hpp"
#include "wingSystem.hpp"

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
              wings(LEFT_WING_PORT, RIGHT_WING_PORT)
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

    private:
        const std::vector<int8_t> L_MOTOR_PORTS = {1};
        const std::vector<int8_t> R_MOTOR_PORTS = {2};

        const uint8_t LEFT_WING_PORT = 1;  // ADI
        const uint8_t RIGHT_WING_PORT = 2; // ADI
        const int8_t CATAPULT_MOTOR_PORT = 5;
        const int8_t INTAKE_MOTOR_PORT = 6;
        const int8_t MANIP_MOTOR_PORT = 7;

        const double WHEEL_RADIUS = 2.0;
        const double WHEEL_BASE = 12.0;
        const double TICKS_PER_REVOLUTION = 360.0;
    };
}