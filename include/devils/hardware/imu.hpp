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
            return Units::degToRad(heading);
        }

        /**
         * Sets the current heading of the IMU in radians.
         * @param heading The heading to set the IMU to in radians.
         */
        double setHeading(double heading)
        {
            double status = imu.set_heading(Units::radToDeg(heading));
            if (status == PROS_ERR && LOGGING_ENABLED)
                Logger::error(name + ": imu set heading failed");
            return status;
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
            return sqrt(pow(accel.x, 2) + pow(accel.y, 2) + pow(accel.z, 2));
        }

    private:
        static constexpr bool LOGGING_ENABLED = true;

        std::string name;
        pros::IMU imu;
    };
}