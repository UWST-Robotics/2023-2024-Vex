#pragma once
#include "pros/adi.hpp"
#include "motor.hpp"
#include "../utils/logger.hpp"
#include <string>

namespace devils
{
    /**
     * Represents a non-V5 pneumatic valve controlled by a legacy motor controller on the ADI ports.
     */
    class ScuffPneumatic
    {
    public:
        /**
         * Creates a new scuff pneumatic.
         * @param name The name of the pneumatic (for logging purposes)
         * @param port The ADI port of the motor controller (from 1 to 8)
         */
        ScuffPneumatic(std::string name, uint8_t port)
            : name(name),
              controller(port)
        {
            if (errno != 0 && LOGGING_ENABLED)
                Logger::error(name + ": adi port is invalid");
        }

        /**
         * Extends the pneumatic.
         */
        void extend()
        {
            int32_t status = controller.set_value(EXTEND_SPEED);
            if (status != 1 && LOGGING_ENABLED)
                Logger::error(name + ": pneumatic extend failed");
        }

        /**
         * Retracts the pneumatic.
         */
        void retract()
        {
            int32_t status = controller.set_value(RETRACT_SPEED);
            if (status != 1 && LOGGING_ENABLED)
                Logger::error(name + ": pneumatic retract failed");
        }

    private:
        static constexpr uint8_t EXTEND_SPEED = 127;
        static constexpr uint8_t RETRACT_SPEED = 0;
        static constexpr bool LOGGING_ENABLED = true;

        std::string name;
        pros::ADIMotor controller;
    };
}