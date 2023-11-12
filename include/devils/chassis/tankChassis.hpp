#pragma once
#include "chassis.hpp"
#include "pros/motors.hpp"
#include <vector>
#include <iostream>

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
            const std::initializer_list<std::int8_t> leftMotorPorts,
            const std::initializer_list<std::int8_t> rightMotorPorts) : leftMotors({pros::Motor(1, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES)}),
                                                                        rightMotors({pros::Motor(2, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_DEGREES)})
        {
            leftMotors.set_gearing(pros::E_MOTOR_GEARSET_18);
            rightMotors.set_gearing(pros::E_MOTOR_GEARSET_18);
            leftMotors.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
            rightMotors.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
        }

        void move(double forward, double turn, double strafe = 0) override
        {
            leftMotors.move(MAX_VOLTAGE * (forward + turn));
            rightMotors.move(MAX_VOLTAGE * (forward - turn));
        }

        bool isHolonomic() override
        {
            return false;
        }

        /**
         * Returns the left motor group.
         * @return The left motor group.
         */
        pros::Motor_Group *getLeftMotors()
        {
            return &leftMotors;
        }

        /**
         * Returns the right motor group.
         * @return The right motor group.
         */
        pros::Motor_Group *getRightMotors()
        {
            return &rightMotors;
        }

    private:
        const int MAX_VOLTAGE = 127;

        pros::MotorGroup leftMotors;
        pros::MotorGroup rightMotors;
    };
}