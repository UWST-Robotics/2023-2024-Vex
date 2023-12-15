#pragma once
#include "../chassis/chassis.hpp"
#include "../path/motionProfile.hpp"
#include "../path/pathFile.hpp"
#include "../odom/odomSource.hpp"
#include "../utils/logger.hpp"
#include "../utils/curve.hpp"
#include "../path/profilePose.hpp"
#include "autoController.hpp"
#include "pros/rtos.hpp"
#include <cmath>
#include <vector>

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
        }

        /**
         * Returns the current target point of the motion profile.
         * Controller tries to choose LOOKAHEAD_DISTANCE inches ahead of the robot.
         * @return The current target point of the motion profile.
         */
        const ProfilePose getCurrentProfilePoint() override
        {
            if (!motionProfile.isGenerated())
                return ProfilePose();

            auto allPoints = motionProfile.getProfilePoints();
            return allPoints[currentPathIndex];
        }

        /**
         * Restarts the motion profile from the beginning.
         */
        void restart() override
        {
            currentPathIndex = 0;
        }

        /**
         * Gets list of current events.
         * @return List of current events.
         */
        const std::vector<PathEvent> getCurrentEvents()
        {
            auto controlPoints = motionProfile.getControlPoints();
            if (currentPointIndex >= controlPoints.size() || currentPointIndex < 0)
                return {};
            return controlPoints[currentPointIndex].events;
        }

        /**
         * Updates the chassis based on the current point of the motion profile.
         * Also updates the chassis input.
         */
        void update() override
        {
            auto currentPose = odometry.getPose();
            auto controlPoints = motionProfile.getControlPoints();
            auto profilePoints = motionProfile.getProfilePoints();

            // Update Closest Motion Profile Point
            double closestDistance = 100000;
            for (int i = currentPathIndex; i < profilePoints.size(); i++)
            {
                auto point = profilePoints[i];
                double distance = abs(LOOKAHEAD_DISTANCE - sqrt(pow(point.x - currentPose.x, 2) + pow(point.y - currentPose.y, 2)));
                if (distance < closestDistance)
                {
                    closestDistance = distance;
                    currentPathIndex = i;
                }
                if (i - currentPathIndex > LOOKAHEAD_MAX_INDICES)
                    break;
            }

            // Update Path Point
            for (int i = currentPointIndex; i < controlPoints.size(); i++)
            {
                auto point = controlPoints[i];
                double xPos = point.x;
                double yPos = point.y;
                double distance = sqrt(pow(xPos - currentPose.x, 2) + pow(yPos - currentPose.y, 2));
                if (distance < EVENT_RANGE)
                    currentPointIndex = i;
                if (i - currentPointIndex > EVENT_MAX_INDICES)
                    break;
            }

            // Get Current Point
            auto currentPathPoint = profilePoints[currentPathIndex];
            auto currentProfilePoint = getCurrentProfilePoint();

            // Calculate Forward & Turn
            double deltaX = currentProfilePoint.x - currentPose.x;
            double deltaY = currentProfilePoint.y - currentPose.y;
            double deltaRotation = Units::diffRad(atan2(deltaY, deltaX), currentPose.rotation);
            double deltaForward = cos(currentPose.rotation) * deltaX + sin(currentPose.rotation) * deltaY;
            double normalForward = deltaForward / LOOKAHEAD_DISTANCE;

            double forward = normalForward * TRANSLATION_SCALE;
            double turn = deltaRotation * ROTATION_SCALE;

            // Clamp Values
            forward = Curve::clamp(-1.0, 1.0, forward) * SPEED_SCALE;
            turn = Curve::clamp(-1.0, 1.0, turn) * SPEED_SCALE;

            chassis.move(forward, turn);
        }

    private:
        static constexpr double LOOKAHEAD_DISTANCE = 3; // in
        static constexpr int LOOKAHEAD_MAX_INDICES = 10;
        static constexpr double TRANSLATION_SCALE = 0.7;
        static constexpr double ROTATION_SCALE = 0.18;
        static constexpr double SPEED_SCALE = 0.9;

        static constexpr double EVENT_RANGE = 12; // in
        static constexpr int EVENT_MAX_INDICES = 10;

        BaseChassis &chassis;
        MotionProfile &motionProfile;
        OdomSource &odometry;
        int currentPathIndex = 0;  // Motion Profile Points
        int currentPointIndex = 0; // Path File Points
    };
}