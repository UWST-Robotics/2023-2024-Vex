#pragma once
#include "pros/motors.hpp"
#include "motor.hpp"
#include "../utils/logger.hpp"
#include <string>

namespace devils
{
    /**
     * Represents a motor object. All events are logged.
     */
    class SmartMotor : public IMotor
    {
    public:
        /**
         * Creates a motor object.
         * @param name The name of the motor (for logging purposes)
         * @param port The port of the motor
         */
        SmartMotor(std::string name, const int8_t port)
            : name(name),
              motor(port)
        {
            if (errno != 0 && LOGGING_ENABLED)
                Logger::error(name + ": motor port is invalid");
        }

        /**
         * Runs the motor in voltage mode.
         * @param voltage The voltage to run the motor at, from -1 to 1.
         */
        void moveVoltage(const double voltage) override
        {
            int32_t status = motor.move(voltage * 127);
            if (status != 1 && LOGGING_ENABLED)
                Logger::error(name + ": motor move failed");
            _checkHealth();
        }

        /**
         * Stops the motor.
         */
        void stop() override
        {
            int32_t status = motor.brake();
            if (status != 1 && LOGGING_ENABLED)
                Logger::error(name + ": motor brake failed");
            _checkHealth();
        }

        /**
         * Gets the current position of the motor in encoder ticks.
         * + 1800 ticks/rev with 36:1 gears (red cartridge)
         * + 900 ticks/rev with 18:1 gears (green cartridge)
         * + 300 ticks/rev with 6:1 gears (blue cartridge)
         * @return The current position of the motor in encoder ticks.
         */
        const double getPosition() override
        {
            double position = motor.get_position();
            if (position == PROS_ERR_F && LOGGING_ENABLED)
                Logger::error(name + ": motor get position failed");
            return position;
        }

        /**
         * Returns the current speed of the motor in RPM.
         */
        const double getSpeed() override
        {
            double velocity = motor.get_actual_velocity();
            if (velocity == PROS_ERR_F && LOGGING_ENABLED)
                Logger::error(name + ": motor get speed failed");
            return velocity;
        }

        /**
         * Checks and logs the current health of the motor. Should be called after every motor command (move, stop, etc).
         */
        void _checkHealth()
        {
            bool isOverTemp = motor.is_over_temp();
            bool isOverCurrent = motor.is_over_current();

            if (isOverTemp && LOGGING_ENABLED)
                Logger::warn(name + ": motor is over temperature");
            if (isOverCurrent && LOGGING_ENABLED)
                Logger::warn(name + ": motor is over current");
        }

    private:
        static constexpr bool LOGGING_ENABLED = true;

        std::string name;
        pros::Motor motor;
    };
}