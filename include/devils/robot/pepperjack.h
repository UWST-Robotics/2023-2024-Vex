#pragma once
#include "devils/utils/logger.h"
#include "devils/chassis/tankChassis.h"
#include "devils/odom/tankWheelOdometry.h"
#include "intakeSystem.h"
#include "wingSystem.h"
#include "climbSystem.h"

namespace devils
{
    class PepperJack
    {
    public:
        /**
         * Represents PepperJack the robot and all of its subsystems.
         */
        PepperJack();

        /**
         * Updates the odometry of the robot.
         */
        void updateOdometry();

        // Subsystems
        TankChassis chassis;
        TankWheelOdometry odometry;
        IntakeSystem intake;
        WingSystem wings;
        ClimbSystem climber;

    private:
        const std::vector<int8_t> L_MOTOR_PORTS = {1, 2, 3, 4};
        const std::vector<int8_t> R_MOTOR_PORTS = {5, 6, 7, 8};

        const uint8_t LEFT_WING_PORT = 1;  // ADI
        const uint8_t RIGHT_WING_PORT = 2; // ADI
        const int8_t INTAKE_MOTOR_PORT = 9;
        const int8_t MANIP_MOTOR_PORT = 10;
        const int8_t CLIMB_MOTOR_PORT = 11;

        const double WHEEL_RADIUS = 2.0;
        const double WHEEL_BASE = 12.0;
        const double TICKS_PER_REVOLUTION = 360.0;
    };
}