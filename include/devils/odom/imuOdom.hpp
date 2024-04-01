#pragma once
#include "odomSource.hpp"
#include "../hardware/imu.hpp"

namespace devils
{

    /**
     * Source of odometry data that double-integrates IMU acceleration to get position.
     * Output needs to be corrected with a separate source of odometry data to accomodate for drift.
     */
    class IMUOdom : public OdomSource
    {
    public:
        IMUOdom(IMU &imu) : imu(imu)
        {
        }

        /**
         * Updates the odometry with the latest data from the IMU
         */
        void update()
        {
            // TODO: Implement
            throw std::runtime_error("Not implemented");
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
         */
        void setPose(Pose &pose) override
        {
            currentPose = pose;
        }

    private:
        IMU &imu;
        Pose currentPose;
    };
}