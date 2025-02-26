#pragma once
#include "chassis.hpp"
#include "../hardware/smartMotorGroup.hpp"
#include <vector>
#include <iostream>
#include "../utils/logger.hpp"

namespace devils
{
    /**
     * Represents a chassis driven by the differential of two motors.
     */
    class TankChassis : public BaseChassis
    {
    public:
        /**
         * Creates a new tank chassis.
         * @param name The name of the chassis (for logging purposes)
         * @param leftMotorPorts The ports of the left motors. Negative ports are reversed.
         * @param rightMotorPorts The ports of the right motors. Negative ports are reversed.
         */
        TankChassis(
            std::string name,
            const std::initializer_list<int8_t> leftMotorPorts,
            const std::initializer_list<int8_t> rightMotorPorts) : leftMotors(name + ".LeftMotors", leftMotorPorts),
                                                                   rightMotors(name + ".RightMotors", rightMotorPorts)
        {
            leftMotors.setBrakeMode(USE_BRAKE_MODE);
            rightMotors.setBrakeMode(USE_BRAKE_MODE);
        }

        /**
         * Runs the chassis in voltage mode.
         * @param forward The forward speed of the robot from -1 to 1.
         * @param turn The turn speed of the robot from -1 to 1.
         */
        void move(double forward, double turn, double strafe = 0) override
        {
            forward *= forwardSpeed;
            turn *= turnSpeed;
            strafe *= strafeSpeed;

            moveTank(forward + turn, forward - turn);
        }

        /**
         * Runs the chassis in voltage mode with individual control of the left and right sides.
         * @param left The speed to run the left side of the chassis from -1 to 1.
         * @param right The speed to run the right side of the chassis from -1 to 1.
         */
        void moveTank(double left, double right)
        {
            double fixedLeft = std::clamp(left, -1.0, 1.0);
            double fixedRight = std::clamp(right, -1.0, 1.0);

            leftMotors.moveVoltage(fixedLeft);
            rightMotors.moveVoltage(fixedRight);
        }

        /**
         * Gets the left motor group.
         * @return The left motor group.
         */
        SmartMotorGroup &getLeftMotors()
        {
            return leftMotors;
        }

        /**
         * Gets the right motor group.
         * @return The right motor group.
         */
        SmartMotorGroup &getRightMotors()
        {
            return rightMotors;
        }

        /**
         * Forces the chassis to stop.
         */
        void stop() override
        {
            leftMotors.stop();
            rightMotors.stop();
        }

    private:
        static constexpr bool USE_BRAKE_MODE = false;

        SmartMotorGroup leftMotors;
        SmartMotorGroup rightMotors;
    };
}