#pragma once
#include "pros/imu.hpp"
#include "../utils/logger.hpp"
#include "../utils/units.hpp"
#include <string>

namespace devils
{
    /**
     * Represents a V5 inertial measurement unit.
     */
    class IMU
    {
    public:
        /**
         * Creates a new IMU.
         * @param name The name of the IMU (for logging purposes)
         * @param port The port of the IMU (from 1 to 21)
         */
        IMU(std::string name, uint8_t port)
            : name(name),
              imu(port)
        {
            imu.reset(true);
            if (errno != 0 && LOGGING_ENABLED)
                Logger::error(name + ": imu port is invalid");
        }

        /**
         * Gets the current heading of the IMU in radians.
         * @return The current heading of the IMU in radians.
         */
        double getHeading()
        {
            double heading = imu.get_heading();
            if (heading == PROS_ERR_F && LOGGING_ENABLED)
                Logger::error(name + ": imu get heading failed");
            return heading == PROS_ERR_F ? 0 : Units::degToRad(heading + headingOffset);
        }

        /**
         * Gets the current pitch of the IMU in radians.
         * @return The current pitch of the IMU in radians.
         */
        double getPitch()
        {
            double pitch = imu.get_pitch();
            if (pitch == PROS_ERR_F && LOGGING_ENABLED)
                Logger::error(name + ": imu get pitch failed");
            return pitch == PROS_ERR_F ? 0 : Units::degToRad(pitch);
        }

        /**
         * Sets the current heading of the IMU in radians.
         * @param heading The heading to set the IMU to in radians.
         */
        void setHeading(double heading)
        {
            headingOffset = Units::radToDeg(heading);
        }

        /**
         * Gets the average acceleration of the IMU in Gs.
         * @return The average acceleration of the IMU in Gs.
         */
        double getAverageAcceleration()
        {
            auto accel = imu.get_accel();
            if (accel.x == PROS_ERR_F && LOGGING_ENABLED)
                Logger::error(name + ": imu get accel failed");
            double avgAcceleration = sqrt(pow(accel.x, 2) + pow(accel.y, 2) + pow(accel.z, 2));
            return accel.x == PROS_ERR_F ? 0 : avgAcceleration;
        }

    private:
        static constexpr bool LOGGING_ENABLED = true;

        double headingOffset = 0;

        std::string name;
        pros::IMU imu;
    };
}