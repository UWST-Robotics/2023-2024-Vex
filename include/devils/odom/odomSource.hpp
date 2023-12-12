#pragma once
#include "odomPose.hpp"

namespace devils
{
    /**
     * Represents a source of odometry data.
     */
    struct OdomSource
    {
    public:
        /**
         * Gets the current pose of the robot
         * @return The current pose of the robot
         */
        virtual const OdomPose getPose() = 0;

        /**
         * Sets the current pose of the robot
         * @param pose The pose to set the robot to
         */
        virtual void setPose(const OdomPose pose) = 0;
    };
}