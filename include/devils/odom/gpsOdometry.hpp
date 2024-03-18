#pragma once
#include "pros/gps.hpp"
#include "../utils/logger.hpp"
#include "../geometry/pose.hpp"
#include "../geometry/units.hpp"
#include "odomSource.hpp"

namespace devils
{
    /**
     * Represents a Vex V5 GPS as an odometry source
     */
    class GPSOdometry : public OdomSource
    {
    public:
        /**
         * Initializes a Vex V5 GPS as an odometry source
         * @param gpsPort The port of the GPS
         */
        GPSOdometry(uint8_t gpsPort) : gps(gpsPort)
        {
            if (errno != 0)
                Logger::error("GPSOdometry: GPS port is invalid");
        }

        /**
         * Updates the odometry with the latest GPS data
         */
        void update()
        {
            auto status = gps.get_status();

            if (status.yaw == PROS_ERR_F)
            {
                Logger::error("GPSOdometry: GPS update failed");
                return;
            }

            currentPose.x = Units::metersToIn(status.x);
            currentPose.y = Units::metersToIn(status.y);
            currentPose.rotation = Units::degToRad(status.yaw);
        }

        /**
         * Gets the current pose of the robot
         * @return The current pose of the robot
         */
        Pose &getPose() override
        {
            return currentPose;
        }

        /**
         * Sets the current pose of the robot
         * @param pose The pose to set the robot to
         * @return The current pose of the robot
         */
        void setPose(Pose &pose) override
        {
            int32_t status = gps.set_position(pose.x, pose.y, pose.rotation);

            if (status != 1)
                Logger::error("GPSOdometry: GPS set position failed");
        }

    private:
        pros::Gps gps;
        Pose currentPose = Pose(0, 0, 0);
    };
}