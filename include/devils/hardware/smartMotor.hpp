#pragma once
#include "pros/motors.hpp"
#include "motor.hpp"
#include "../utils/logger.hpp"
#include "../utils/ramp.hpp"
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
         * @param port The port of the motor (from 1 to 21)
         */
        SmartMotor(std::string name, int8_t port)
            : name(name),
              motor(port),
              voltageRamp(100000)
        {
            if (errno != 0 && LOGGING_ENABLED)
                Logger::error(name + ": motor port is invalid");
        }

        /**
         * Runs the motor in voltage mode.
         * @param voltage The voltage to run the motor at, from -1 to 1.
         */
        void moveVoltage(double voltage) override
        {
            // Move Motor
            int32_t status = motor.move(voltageRamp.update(voltage) * 127);
            if (status != 1 && LOGGING_ENABLED)
                Logger::error(name + ": motor move failed");
            _checkHealth();
        }

        /**
         * Sets the ramp rate of the motor.
         * @param rampRate The ramp rate of the motor from 0 to 2.
         */
        void setRampRate(double rampRate)
        {
            voltageRamp.setRampRate(rampRate);
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
        double getPosition() override
        {
            double position = motor.get_position();
            if (position == PROS_ERR_F && LOGGING_ENABLED)
                Logger::error(name + ": motor get position failed");
            return position == PROS_ERR_F ? 0 : position;
        }

        /**
         * Returns the current speed of the motor in RPM.
         */
        double getSpeed() override
        {
            double velocity = motor.get_actual_velocity();
            if (velocity == PROS_ERR_F && LOGGING_ENABLED)
                Logger::error(name + ": motor get speed failed");
            return velocity == PROS_ERR_F ? 0 : velocity;
        }

        /**
         * Checks and logs the current health of the motor. Should be called after every motor command (move, stop, etc).
         */
        void _checkHealth()
        {
            int32_t isOverTemp = motor.is_over_temp();
            int32_t isOverCurrent = motor.is_over_current();

            if ((isOverTemp == PROS_ERR || isOverCurrent == PROS_ERR) && LOGGING_ENABLED)
                Logger::warn(name + ": motor health check failed");
            else if (isOverTemp == 1 && LOGGING_ENABLED)
                Logger::warn(name + ": motor is over temperature");
            else if (isOverCurrent == 1 && LOGGING_ENABLED)
                Logger::warn(name + ": motor is over current");
        }

    private:
        static constexpr bool LOGGING_ENABLED = true;

        double currentVoltage = 0;
        std::string name;
        pros::Motor motor;
        Ramp voltageRamp;
    };
}