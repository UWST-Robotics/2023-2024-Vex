#pragma once
#include "../chassis/tankChassis.hpp"
#include "../hardware/imu.hpp"
#include "../hardware/rotationSensor.hpp"
#include "../utils/logger.hpp"
#include "../geometry/pose.hpp"
#include "odomSource.hpp"
#include "pros/rtos.hpp"
#include "pros/error.h"
#include <cmath>
#include <errno.h>

namespace devils
{
    /**
     * Represents an odometry system that transforms another odometry system as needed.
     */
    class TransformOdom : public OdomSource
    {
    public:
        /**
         * Creates a new transform odometry system.
         * @param odomSource The odometry source to transform.
         * @param mirrorX Whether to mirror the X axis.
         * @param mirrorY Whether to mirror the Y axis.
         */
        TransformOdom(OdomSource &odomSource, bool mirrorX = false, bool mirrorY = false)
            : odomSource(odomSource),
                mirrorX(mirrorX),
                mirrorY(mirrorY)
        {
        }

        /**
         * Gets the current pose of the robot.
         */
        Pose &getPose() override
        {
            Pose &pose = odomSource.getPose();
            if (mirrorX) {
                pose.x = -pose.x;
                pose.rotation = Units::normalizeRadians(M_PI - pose.rotation);
            }
            if (mirrorY) {
                pose.y = -pose.y;
                pose.rotation = Units::normalizeRadians(-pose.rotation);
            }
            return pose;
        }

        /**
         * Sets the current pose of the robot.
         * @param pose The pose to set the robot to.
         */
        void setPose(Pose &pose) override
        {
            Pose newPose = pose;
            if (mirrorX)
                newPose.x = -newPose.x;
            if (mirrorY)
                newPose.y = -newPose.y;
            odomSource.setPose(pose);
        }

        /**
         * Sets whether to mirror the X axis.
         * @param mirrorX Whether to mirror the X axis.
        */
        void setMirrorX(bool mirrorX)
        {
            this->mirrorX = mirrorX;
        }

        /**
         * Sets whether to mirror the Y axis.
         * @param mirrorY Whether to mirror the Y axis.
        */
        void setMirrorY(bool mirrorY)
        {
            this->mirrorY = mirrorY;
        }

    private:
        OdomSource &odomSource;

        bool mirrorX = false;
        bool mirrorY = false;
    };
}