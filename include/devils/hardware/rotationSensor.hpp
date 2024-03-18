#pragma once
#include "pros/imu.hpp"
#include "../utils/logger.hpp"
#include "../geometry/units.hpp"
#include <string>

namespace devils
{
    /**
     * Represents a V5 rotational sensor.
     */
    class RotationSensor
    {
    public:
        /**
         * Creates a new IMU.
         * @param name The name of the rotational sensor (for logging purposes)
         * @param port The port of the rotational sensor (from 1 to 21)
         * @param isReversed Whether the sensor is reversed
         */
        RotationSensor(std::string name, uint8_t port, bool isReversed = false)
            : name(name),
              rotationSensor(port, isReversed)
        {
            if (errno != 0 && LOGGING_ENABLED)
                Logger::error(name + ": rotationSensor port is invalid");
        }

        /**
         * Gets the absolute angle of the sensor in radians.
         * @return The absolute angle of the sensor in radians.
         */
        double getAngle()
        {
            double angle = rotationSensor.get_angle();
            if (angle == PROS_ERR && LOGGING_ENABLED)
                Logger::error(name + ": rotation sensor get angle failed");
            return angle == PROS_ERR ? 0 : Units::centidegToRad(angle);
        }

        /**
         * Gets the velocity of the sensor in radians per second.
         * @return The velocity of the sensor in radians per second.
         */
        double getVelocity()
        {
            double velocity = rotationSensor.get_velocity();
            if (velocity == PROS_ERR_F && LOGGING_ENABLED)
                Logger::error(name + ": rotation sensor get velocity failed");
            return velocity == PROS_ERR_F ? 0 : Units::centidegToRad(velocity);
        }

    private:
        static constexpr bool LOGGING_ENABLED = true;

        std::string name;
        pros::Rotation rotationSensor;
    };
}