#pragma once
#include "pros/imu.hpp"
#include "../utils/logger.hpp"
#include "../geometry/units.hpp"
#include "../geometry/vector3.hpp"
#include "../odom/odomSource.hpp"
#include <string>

namespace devils
{
    /**
     * Represents a V5 inertial measurement unit.
     */
    class IMU : public OdomSource
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
            if (CALIBRATE_ON_START)
                imu.reset(true);
            if (errno != 0 && LOGGING_ENABLED)
                Logger::error(name + ": imu port is invalid");
        }

        /**
         * Converts heading to an `OdomSource` at (0, 0).
         * @param heading The heading to convert to an `OdomSource`.
         */
        Pose &getPose() override
        {
            odomPose.rotation = getHeading();
            return odomPose;
        }

        /**
         * Sets the heading of the IMU from a given `Pose`.
         * @param pose The pose to set the IMU to. Only uses `Pose.rotation`.
         */
        void setPose(Pose &pose) override
        {
            setHeading(pose.rotation);
        }

        /**
         * Gets the current acceleration of the IMU in inches per second squared.
         * @return The current acceleration of the IMU in inches per second squared.
         */
        Vector3 getAccel()
        {
            auto accel = imu.get_accel();
            if (accel.x == PROS_ERR_F && LOGGING_ENABLED)
            {
                Logger::error(name + ": imu get accel failed");
                return Vector3(0, 0, 0);
            }
            return Vector3(
                Units::metersToIn(accel.x),
                Units::metersToIn(accel.y),
                Units::metersToIn(accel.z));
        }

        /**
         * Gets the current heading of the IMU in radians.
         * @return The current heading of the IMU in radians or `PROS_ERR_F` if the operation failed.
         */
        double getHeading()
        {
            double heading = imu.get_heading();
            if (heading == PROS_ERR_F && LOGGING_ENABLED)
                Logger::error(name + ": imu get heading failed");
            return heading == PROS_ERR_F ? PROS_ERR_F : Units::degToRad(heading);
        }

        /**
         * Gets the current pitch of the IMU in radians.
         * @return The current pitch of the IMU in radians or `PROS_ERR_F` if the operation failed.
         */
        double getPitch()
        {
            double pitch = imu.get_pitch();
            if (pitch == PROS_ERR_F && LOGGING_ENABLED)
                Logger::error(name + ": imu get pitch failed");
            return pitch;
        }

        /**
         * Sets the current heading of the IMU in radians.
         * @param heading The heading to set the IMU to in radians.
         */
        void setHeading(double heading)
        {
            int32_t result = imu.set_heading(Units::radToDeg(heading + headingOffset));
            if (result == PROS_ERR && LOGGING_ENABLED)
                Logger::error(name + ": imu set heading failed");
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

        /**
         * Calibrates the IMU. Robot should be still during calibration.
         * Run `waitUntilCalibrated` to wait until calibration is finished.
         */
        void calibrate()
        {
            imu.reset(false);
        }

        /**
         * Waits until the IMU is finished calibrating.
         * Should be ran to avoid movement during calibration.
         */
        void waitUntilCalibrated()
        {
            while (imu.is_calibrating())
                pros::delay(20);
        }

    private:
        static constexpr bool CALIBRATE_ON_START = false;
        static constexpr bool LOGGING_ENABLED = true;

        double headingOffset = 0;

        std::string name;
        pros::IMU imu;
        Pose odomPose;
    };
}