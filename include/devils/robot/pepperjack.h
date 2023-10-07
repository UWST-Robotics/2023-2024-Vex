#pragma once
#include "devils/utils/logger.h"
#include "devils/chassis/tankChassis.h"
#include "devils/odom/tankWheelOdometry.h"
#include "intakeSystem.h"
#include "wingSystem.h"

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

    private:
        const uint8_t FL_MOTOR_PORT = 1;
        const uint8_t BL_MOTOR_PORT = 2;
        const uint8_t FR_MOTOR_PORT = 3;
        const uint8_t BR_MOTOR_PORT = 4;

        const uint8_t INTAKE_MOTOR_PORT = 6;
        const uint8_t MANIP_MOTOR_PORT = 7;
        const uint8_t LEFT_WING_PORT = 1;
        const uint8_t RIGHT_WING_PORT = 2;

        const double WHEEL_RADIUS = 2.0;
        const double WHEEL_BASE = 12.0;
        const double TICKS_PER_REVOLUTION = 360.0;
    };
}