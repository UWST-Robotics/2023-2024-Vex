#pragma once
#include "pros/gps.hpp"
#include "odomPose.hpp"
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
        const OdomPose getPose() override
        {
            return currentPose;
        }

        /**
         * Sets the current pose of the robot
         * @param pose The pose to set the robot to
         * @return The current pose of the robot
         */
        void setPose(const OdomPose pose) override
        {
            currentPose = pose;
        }

    private:
        OdomPose currentPose;
    };
}