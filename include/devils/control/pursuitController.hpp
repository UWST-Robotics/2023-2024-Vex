#pragma once
#include "../chassis/chassis.hpp"
#include "../path/motionProfile.hpp"
#include "../odom/odomSource.hpp"
#include "autoController.hpp"
#include "pros/rtos.hpp"
#include "../utils/logger.hpp"
#include <cmath>

namespace devils
{
    class PursuitController : public AutoController
    {
    public:
        PursuitController(BaseChassis &chassis, MotionProfile &motionProfile, OdomSource &odometry)
            : chassis(chassis), motionProfile(motionProfile), odometry(odometry)
        {
            motionProfile.generate();
        }

        squiggles::ProfilePoint getCurrentPoint() override
        {
            if (!motionProfile.isGenerated())
                return squiggles::ProfilePoint();

            auto allPoints = motionProfile.getPath();
            return allPoints[minimumIndex];
        }

        void restart() override
        {
            minimumIndex = 0;
        }

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

            chassis.move(forward, turn);
        }

    private:
        static constexpr double LOOKAHEAD_DISTANCE = 24; // in
        static constexpr double LOOKAHEAD_MAX_INDICES = 20;
        static constexpr double TRANSLATION_SCALE = 0.04;
        static constexpr double ROTATION_SCALE = 1;

        int minimumIndex = 0;
        BaseChassis &chassis;
        MotionProfile &motionProfile;
        OdomSource &odometry;
    };
}