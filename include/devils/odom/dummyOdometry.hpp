#pragma once
#include "pros/gps.hpp"
#include "pose.hpp"
#include "../utils/logger.hpp"
#include "odomSource.hpp"

namespace devils
{
    /**
     * Represents a dummy odometry source
     */
    class DummyOdometry : public OdomSource
    {
    public:
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
        void setPose(const Pose pose) override
        {
            currentPose = pose;
        }

    private:
        Pose currentPose;
    };
}