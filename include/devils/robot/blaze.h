#pragma once
#include "devils/utils/logger.h"
#include "devils/chassis/tankChassis.h"
#include "devils/odom/tankWheelOdometry.h"
#include "catapultSystem.h"
#include "intakeSystem.h"
#include "wingSystem.h"

namespace devils
{
    struct Blaze
    {
    public:
        /**
         * Represents Blaze the robot and all of its subsystems.
         */
        Blaze();

        /**
         * Updates the odometry of the robot.
         */
        void updateOdometry();

        // Subsystems
        TankChassis chassis;
        TankWheelOdometry odometry;
        CatapultSystem catapult;
        IntakeSystem intake;
        WingSystem wings;

    private:
        const std::vector<int8_t> L_MOTOR_PORTS = {1, 2, 3, 4};
        const std::vector<int8_t> R_MOTOR_PORTS = {5, 6, 7, 8};

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