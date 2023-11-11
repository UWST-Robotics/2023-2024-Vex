#pragma once
#include "pros/gps.hpp"
#include "pose.hpp"
#include "../utils/logger.hpp"
#include "odomSource.hpp"

namespace devils
{
    class GPSOdometry : public OdomSource
    {
    public:
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
            if (errno != 0)
                Logger::error("GPSOdometry: GPS update failed");
            currentPose.x = status.x;
            currentPose.y = status.y;
            currentPose.rotation = status.yaw;
        }

        /**
         * Gets the current pose of the robot
         * @return The current pose of the robot
         */
        const Pose getPose() override
        {
            return currentPose;
        }

        /**
         * Sets the current pose of the robot
         * @param pose The pose to set the robot to
         * @return The current pose of the robot
         */
        void setPose(Pose pose) override
        {
            gps.set_position(pose.x, pose.y, pose.rotation);
            if (errno != 0)
                Logger::error("GPSOdometry: GPS set position failed");
        }

        /**
         * Returns true if the robot is out of bounds
         * @return True if the robot is out of bounds
         */
        const bool isOutOfBounds()
        {
            // TODO: Check out of bounds
            return false;
        }

    private:
        pros::Gps gps;
        Pose currentPose;
    };
}