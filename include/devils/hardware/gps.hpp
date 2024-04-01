#pragma once
#include "pros/gps.hpp"
#include "../utils/logger.hpp"
#include "../geometry/pose.hpp"
#include "../geometry/units.hpp"
#include "../odom/odomSource.hpp"

namespace devils
{
    /**
     * Represents a Vex V5 GPS
     */
    class GPS : public OdomSource
    {
    public:
        /**
         * Initializes a Vex V5 GPS as an odometry source.
         * Remember to call `GPS::setPosition` to set the initial position before the GPS locks on.
         * Also call `GPS::setOffset` to set the GPS's position relative to the center of the robot.
         * @param gpsPort The port of the GPS
         */
        GPS(std::string name, uint8_t gpsPort)
            : name(name),
              gps(gpsPort)
        {
            if (errno != 0)
                Logger::error(name + ": GPS port is invalid");
        }

        /**
         * Updates the odometry with the latest GPS data
         */
        void update()
        {
            auto status = gps.get_status();

            if (status.yaw == PROS_ERR_F)
            {
                Logger::error(name + ": GPS update failed");
                return;
            }

            Pose gpsPose = Pose(
                Units::metersToIn(status.x),
                Units::metersToIn(status.y),
                Units::normalizeRadians(Units::degToRad(status.yaw) - GPS_ROTATION_OFFSET));

            currentPose = gpsPose - gpsOffset;
        }

        /**
         * Gets the current pose of the robot since the last `GPS::update`
         * @return The current pose of the robot
         */
        Pose &getPose() override
        {
            return currentPose;
        }

        /**
         * Sets the current pose of the robot.
         * This value is overriden when the GPS locks on to the field strip.
         * @param pose The pose to set the robot to
         * @return The current pose of the robot
         */
        void setPose(Pose &pose) override
        {
            currentPose = pose;

            double gpsX = Units::inToMeters(pose.x + gpsOffset.x);
            double gpsY = Units::inToMeters(pose.y + gpsOffset.y);
            double gpsYaw = Units::degToRad(pose.rotation + GPS_ROTATION_OFFSET + gpsOffset.rotation);

            int32_t status = gps.set_position(
                gpsX,
                gpsY,
                Units::normalizeRadians(gpsYaw));

            if (status != 1)
                Logger::error(name + ": GPS set position failed");
        }

        /**
         * Sets the GPS's position relative to the center of the robot.
         * @param x The x offset of the GPS from the center of the robot in inches
         * @param y The y offset of the GPS from the center of the robot in inches
         * @param rotation The rotation offset of the GPS from the center of the robot in radians
         */
        void setOffset(double x, double y, double rotation)
        {
            gpsOffset = Pose(x, y, rotation);
        }

    private:
        static constexpr double GPS_ROTATION_OFFSET = M_PI * 0.5; // PROS defaults to north as 0 degrees

        std::string name;
        pros::Gps gps;
        Pose currentPose = Pose(0, 0, 0);
        Pose gpsOffset = Pose(0, 0, 0);
    };
}