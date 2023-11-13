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
         * @param name The name of the motor group (for logging purposes)
         * @param ports The ports of the motors in the group (from 1 to 21)
         */
        SmartMotorGroup(const std::string name, const std::initializer_list<int8_t> ports)
            : name(name), motors()
        {
            motors.reserve(ports.size());
            for (int8_t port : ports)
                motors.push_back(std::make_shared<SmartMotor>(_getMotorName(port), port));
        }

        /**
         * Runs all the motors in voltage mode.
         * @param voltage The voltage to run the motors at, from -1 to 1.
         */
        void moveVoltage(const double voltage) override
        {
            for (auto motor : motors)
                motor->moveVoltage(voltage);
        }

        /**
         * Sets the ramp rate of the motor group.
         * @param rampRate The max change in motor value per second.
         */
        void setRampRate(const double rampRate)
        {
            for (auto motor : motors)
                motor->setRampRate(rampRate);
        }

        /**
         * Stops all the motors.
         */
        void stop() override
        {
            for (auto motor : motors)
                motor->stop();
        }

        /**
         * Gets the average position of all the motors in encoder ticks.
         * @return The average position of all the motors in encoder ticks.
         */
        const double getPosition() override
        {
            double position = 0;
            for (auto motor : motors)
                position += motor->getPosition();
            return position / motors.size();
        }

        /**
         * Returns the average speed of all the motors in RPM.
         * @return The average speed of all the motors in RPM.
         */
        const double getSpeed() override
        {
            double speed = 0;
            for (auto motor : motors)
                speed += motor->getSpeed();
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
        std::vector<std::shared_ptr<SmartMotor>> motors;
    };
}