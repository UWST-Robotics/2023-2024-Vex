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
         * @param leftMotorPorts The ports of the left motors. Negative ports are reversed.
         * @param rightMotorPorts The ports of the right motors. Negative ports are reversed.
         */
        TankChassis(
            const std::initializer_list<int8_t> leftMotorPorts,
            const std::initializer_list<int8_t> rightMotorPorts) : leftMotors("TankChassis.Left Motors", leftMotorPorts),
                                                                   rightMotors("TankChassis.Right Motors", rightMotorPorts)
        {
        }

        /**
         * Runs the chassis in voltage mode.
         * @param forward The forward speed of the robot from -1 to 1.
         * @param turn The turn speed of the robot from -1 to 1.
         */
        void move(double forward, double turn, double strafe = 0) override
        {
            leftMotors.moveVoltage(forward + turn);
            rightMotors.moveVoltage(forward - turn);
        }

        /**
         * Returns whether or not the chassis is holonomic.
         * @return false
         */
        bool isHolonomic() override
        {
            return false;
        }

        /**
         * Gets the left motor group.
         * @return The left motor group.
         */
        SmartMotorGroup *getLeftMotors()
        {
            return &leftMotors;
        }

        /**
         * Gets the right motor group.
         * @return The right motor group.
         */
        SmartMotorGroup *getRightMotors()
        {
            return &rightMotors;
        }

        /**
         * Forces the chassis to stop.
         */
        void stop()
        {
            leftMotors.stop();
            rightMotors.stop();
        }

    private:
        SmartMotorGroup leftMotors;
        SmartMotorGroup rightMotors;
    };
}