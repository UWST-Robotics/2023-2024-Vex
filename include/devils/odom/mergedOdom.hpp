#pragma once
#include "pros/gps.hpp"
#include "pose.hpp"
#include "../utils/logger.hpp"
#include "odomSource.hpp"
#include "../utils/units.hpp"

namespace devils
{
    /**
     * Represents a set of odometry sources merged into one
     */
    class MergedOdom : public OdomSource
    {
    public:
        /**
         * Merged odometry from multiple sources
         * @param absoluteOdom An absolute odometry source, such as GPS
         * @param relativeOdom A relative odometry source, such as differential wheel odometry or an IMU
         */
        MergedOdom(OdomSource *absoluteOdom,
                   OdomSource *relativeOdom)
        {
            this->absoluteOdom = absoluteOdom;
            this->relativeOdom = relativeOdom;
        }

        /**
         * Updates the odometry with the latest data from the sources
         */
        void update()
        {
            // TODO: Kalman filter between absolute and relative odometry

            Pose &absolutePose = absoluteOdom->getPose();

            bool hasAbsoluteUpdated = absolutePose != lastAbsolutePose;
            if (hasAbsoluteUpdated)
                relativeOdom->setPose(absolutePose);

            lastAbsolutePose = absolutePose;
        }

        /**
         * Gets the current pose of the robot
         * @return The current pose of the robot
         */
        Pose &getPose() override
        {
            return relativeOdom->getPose();
        }

        /**
         * Sets the current pose of the robot
         * @param pose The pose to set the robot to
         */
        void setPose(Pose &pose) override
        {
            relativeOdom->setPose(pose);
        }

    private:
        OdomSource *absoluteOdom;
        OdomSource *relativeOdom;

        Pose lastAbsolutePose = Pose();
    };
}