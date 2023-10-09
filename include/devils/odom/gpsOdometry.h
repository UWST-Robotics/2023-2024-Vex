#pragma once
#include "pros/gps.hpp"
#include "pose.h"

namespace devils
{
    class GPSOdometry
    {
    public:
        GPSOdometry(uint8_t gpsPort);

        /**
         * Updates the odometry with the latest GPS data
         */
        void update();

        /**
         * Gets the current pose of the robot
         * @return The current pose of the robot
         */
        const Pose getPose();

        /**
         * Sets the current pose of the robot
         * @param pose The pose to set the robot to
         * @return The current pose of the robot
         */
        void setPose(Pose pose);

    private:
        pros::Gps gps;
        Pose currentPose;
    };
}