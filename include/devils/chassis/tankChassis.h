#pragma once
#include "chassis.h"
#include "pros/motors.hpp"
#include <vector>

namespace devils
{
    class TankChassis : public BaseChassis
    {
    public:
        /**
         * Represents a tank drive chassis.
         * @param leftMotorPorts The ports of the left motors. Negative ports are reversed.
         * @param rightMotorPorts The ports of the right motors. Negative ports are reversed.
         */
        TankChassis(
            const std::vector<int8_t> leftMotorPorts,
            const std::vector<int8_t> rightMotorPorts);

        void move(double forward, double turn, double strafe = 0) override;
        bool isHolonomic() override;

        /**
         * Returns the left motor group.
         * @return The left motor group.
         */
        pros::Motor_Group *getLeftMotors();

        /**
         * Returns the right motor group.
         * @return The right motor group.
         */
        pros::Motor_Group *getRightMotors();

    private:
        const int MAX_VOLTAGE = 127;

        pros::MotorGroup leftMotors;
        pros::MotorGroup rightMotors;
    };
}