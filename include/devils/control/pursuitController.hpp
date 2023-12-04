#pragma once
#include "../chassis/chassis.hpp"
#include "../path/motionProfile.hpp"
#include "../odom/odomSource.hpp"
#include "../utils/logger.hpp"
#include "../utils/curve.hpp"
#include "autoController.hpp"
#include "pros/rtos.hpp"
#include <cmath>

namespace devils
{
    /**
     * Controller for follwing a motion profile with sensor feedback using Basic Pure Pursuit.
     */
    class PursuitController : public AutoController
    {
    public:
        /**
         * Constructs a new PursuitController.
         * @param chassis The chassis to control.
         * @param motionProfile The motion profile to follow.
         */
        PursuitController(BaseChassis &chassis, MotionProfile &motionProfile, OdomSource &odometry)
            : chassis(chassis), motionProfile(motionProfile), odometry(odometry)
        {
            motionProfile.generate();
        }

        /**
         * Returns the current target point of the motion profile.
         * Controller tries to choose LOOKAHEAD_DISTANCE inches ahead of the robot.
         * @return The current target point of the motion profile.
         */
        const squiggles::ProfilePoint getCurrentPoint() override
        {
            if (!motionProfile.isGenerated())
                return squiggles::ProfilePoint();

            auto allPoints = motionProfile.getPath();
            return allPoints[minimumIndex];
        }

        /**
         * Restarts the motion profile from the beginning.
         */
        void restart() override
        {
            minimumIndex = 0;
        }

        /**
         * Updates the chassis based on the current point of the motion profile.
         * Also updates the chassis input.
         */
        void update() override
        {
            // Update Closest Point
            auto currentPose = odometry.getPose();
            auto allPoints = motionProfile.getPath();
            auto previousPoint = getCurrentPoint();

            double closestDistance = 100000;
            for (int i = minimumIndex; i < allPoints.size(); i++)
            {
                auto point = allPoints[i];
                double xPos = Units::metersToIn(point.vector.pose.x);
                double yPos = Units::metersToIn(point.vector.pose.y);
                double distance = abs(LOOKAHEAD_DISTANCE - sqrt(pow(xPos - currentPose.x, 2) + pow(yPos - currentPose.y, 2)));
                if (distance < closestDistance)
                {
                    closestDistance = distance;
                    minimumIndex = i;
                }
                if (i - minimumIndex > LOOKAHEAD_MAX_INDICES)
                    break;
            }

            // Get Current Point
            auto currentPoint = getCurrentPoint();

            // Calculate Forward & Turn
            double deltaX = Units::metersToIn(currentPoint.vector.pose.x) - currentPose.x;
            double deltaY = Units::metersToIn(currentPoint.vector.pose.y) - currentPose.y;
            double deltaRotation = Units::diffRad(atan2(deltaY, deltaX), currentPose.rotation);
            double deltaForward = cos(currentPose.rotation) * deltaX + sin(currentPose.rotation) * deltaY;

            double forward = deltaForward * TRANSLATION_SCALE;
            double turn = deltaRotation * ROTATION_SCALE;

            // Clamp Values
            forward = Curve::clamp(-1.0, 1.0, forward) * 0.5;
            turn = Curve::clamp(-1.0, 1.0, turn) * 0.5;

            chassis.move(forward, turn);
        }

    private:
        static constexpr double LOOKAHEAD_DISTANCE = 24; // in
        static constexpr double LOOKAHEAD_MAX_INDICES = 20;
        static constexpr double TRANSLATION_SCALE = 0.03;
        static constexpr double ROTATION_SCALE = 0.9;

        int minimumIndex = 0;
        BaseChassis &chassis;
        MotionProfile &motionProfile;
        OdomSource &odometry;
    };
}