#pragma once
#include "pros/gps.hpp"
#include "../utils/logger.hpp"
#include "../geometry/pose.hpp"
#include "../geometry/units.hpp"
#include "../geometry/polygon.hpp"
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
              gps(gpsPort, 0, 0, 0, 0, 0)
        {
            if (errno != 0)
                Logger::error(name + ": GPS port is invalid");
            gps.set_data_rate(40);
        }

        /**
         * Updates the odometry with the latest GPS data
         */
        void update()
        {
            double gpsX = gps.get_x_position();
            double gpsY = gps.get_y_position();
            double gpsHeading = gps.get_heading();

            if (gpsX == PROS_ERR_F || gpsY == PROS_ERR_F || gpsHeading == PROS_ERR_F)
            {
                Logger::error(name + ": GPS update failed");
                return;
            }

            // Convert Units
            gpsX = Units::metersToIn(gpsX);
            gpsY = -Units::metersToIn(gpsY);
            gpsHeading = Units::normalizeRadians(Units::degToRad(gpsHeading) - GPS_ROTATION_OFFSET - rotationalOffset);

            // Check Within Bounds
            if (gpsX < -MAX_GPS_X || gpsX > MAX_GPS_X || gpsY < -MAX_GPS_Y || gpsY > MAX_GPS_Y)
            {
                Logger::error(name + ": GPS out of bounds");
                return;
            }

            // Update Pose
            currentPose.x = gpsX;
            currentPose.y = gpsY;
            currentPose.rotation = gpsHeading;
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

            double gpsX = Units::inToMeters(pose.x);
            double gpsY = Units::inToMeters(pose.y);
            double gpsYaw = Units::degToRad(pose.rotation) + GPS_ROTATION_OFFSET + rotationalOffset;

            int32_t status = gps.set_position(
                gpsX,
                gpsY,
                gpsYaw);

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
            int32_t result = gps.set_offset(
                Units::inToMeters(x),
                Units::inToMeters(y));
            rotationalOffset = rotation;

            if (result != 1)
                Logger::error(name + ": GPS set offset failed");
        }

    private:
        static constexpr double GPS_ROTATION_OFFSET = M_PI * 0.5; // PROS defaults to north as 0 degrees
        static constexpr double MAX_GPS_X = 72;
        static constexpr double MAX_GPS_Y = 72;

        std::string name;
        pros::Gps gps;
        Pose currentPose = Pose(0, 0, 0);
        double rotationalOffset = 0;
    };
}