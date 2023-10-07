#pragma once
#include "chassis.h"
#include "pros/motors.hpp"
#include <vector>

namespace devils
{
    class TankChassis : public BaseChassis
    {
    public:
        TankChassis(
            const std::vector<uint8_t> leftMotorPorts,
            const std::vector<uint8_t> rightMotorPorts);

        void move(double forward, double turn, double strafe = 0) override;
        bool isHolonomic() override;

        pros::Motor_Group *getLeftMotors();
        pros::Motor_Group *getRightMotors();

    private:
        const int MAX_VOLTAGE = 127;

        pros::MotorGroup leftMotors;
        pros::MotorGroup rightMotors;
    };
}