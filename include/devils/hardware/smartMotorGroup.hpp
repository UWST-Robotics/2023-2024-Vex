#pragma once
#include <string>
#include "motor.hpp"
#include "smartMotor.hpp"

namespace devils
{
    /**
     * Represents a set of smart motors grouped together.
     */
    class SmartMotorGroup : public IMotor
    {
    public:
        /**
         * Creates a new smart motor group.
         * @param name The name of the motor group.
         * @param ports The ports of the motors in the group.
         */
        SmartMotorGroup(const std::string name, const std::initializer_list<int8_t> ports)
            : name(name), motors()
        {
            motors.reserve(ports.size());
            for (int8_t port : ports)
                motors.push_back(SmartMotor(_getMotorName(port), port));
        }

        /**
         * Runs all the motors in voltage mode.
         * @param voltage The voltage to run the motors at, from -1 to 1.
         */
        void moveVoltage(const double voltage) override
        {
            for (SmartMotor motor : motors)
                motor.moveVoltage(voltage);
        }

        /**
         * Stops all the motors.
         */
        void stop() override
        {
            for (SmartMotor motor : motors)
                motor.stop();
        }

        /**
         * Gets the average position of all the motors in encoder ticks.
         * @return The average position of all the motors in encoder ticks.
         */
        const double getPosition() override
        {
            double position = 0;
            for (SmartMotor motor : motors)
                position += motor.getPosition();
            return position / motors.size();
        }

        /**
         * Returns the average speed of all the motors in RPM.
         * @return The average speed of all the motors in RPM.
         */
        const double getSpeed() override
        {
            double speed = 0;
            for (SmartMotor motor : motors)
                speed += motor.getSpeed();
            return speed / motors.size();
        }

        /**
         * Gets the name of each motor in the motor group.
         */
        const std::string _getMotorName(int32_t port)
        {
            return name + "_" + std::to_string(port);
        }

    private:
        const std::string name;
        std::vector<SmartMotor> motors;
    };
}